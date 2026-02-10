#include "lf_hitag2_data.h"

#include "bsp_delay.h"
#include "bsp_time.h"
#include "circular_buffer.h"
#include "hw_connect.h"
#include "lf_125khz_radio.h"
#include "lf_reader_data.h"
#include "protocols/hitag.h"
#include "timeslot.h"
#include "nrfx_saadc.h"

#define NRF_LOG_MODULE_NAME hitag2_reader
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
NRF_LOG_MODULE_REGISTER();

// External PWM sequence for soft pulse control
extern nrf_pwm_values_individual_t m_lf_125khz_pwm_seq_val[];

// Hitag2 protocol timing constants (in microseconds)
// Based on Proxmark3 hitag2.c and Hitag2 specification
// At 125kHz: 1 carrier period (Tc) = 8μs
#define HITAG_T_WAIT_POWERUP_US    2504  // 313 Tc = 2.504ms - Tag powerup time
#define HITAG_T_WAIT_START_AUTH_US  464  // 58 Tc = 464μs - START_AUTH window
#define HITAG_T_WAIT_RESPONSE_US   5000  // EXTENDED: Was 1600µs, now 5ms for slow/weak responses

// START_AUTH command: 5 bits = 11000 binary (MSB first)
#define HITAG2_START_AUTH_BITS    5
#define HITAG2_START_AUTH_CMD     0xC0  // 11000xxx in binary

// Buffer size calculation:
// PWM at 125kHz = 125,000 samples/second
// 100ms collection = 12,500 samples needed
// Use 16384 (power of 2) for full UID capture with margin
// CRITICAL: 8192 was too small - only captured ~30 edges (need ~70+)
#define HITAG2_BUFFER_SIZE        16384  // Doubled for full 32-bit UID capture

// Calibrated threshold based on scope measurements
// Scope data: Floor=0V, Weak data peaks=2.25V, Strong peaks=3.28V
// Option 1: Midpoint between floor and weak peaks = (0V + 2.25V) / 2 = 1.125V
// ADC value (12-bit, 3.3V ref): (1.125 / 3.3) × 4095 = 1395 ADC
// Margin above threshold: 2793 - 1395 = 1398 ADC (1.125V) - Much safer!
#define HITAG2_ADC_THRESHOLD_CALIBRATED 1395  // 1.125V for reliable 2.25V weak peak detection

static circular_buffer cb;

/**
 * Set LF PWM duty cycle for soft pulse control
 * @param duty Duty cycle value (0 = 0%, 2 = 50% with top_value=4)
 */
static void set_lf_pwm_duty(uint16_t duty) {
    m_lf_125khz_pwm_seq_val[0].channel_0 = duty;
}

// SAADC callback for receiving tag response (analog sampling)
// Similar to HID implementation but for Hitag2 response detection
static void hitag2_saadc_cb(nrf_saadc_value_t *vals, size_t size) {
    for (int i = 0; i < size; i++) {
        nrf_saadc_value_t val = vals[i];
        if (!cb_push_back(&cb, &val)) {
            return;
        }
    }
}

static void init_hitag2_hw(void) {
    // Use SAADC for analog sampling (like HID) instead of GPIO interrupts
    // This avoids timeslot GPIOTE lockout issue and works with weak signals (0.434V-0.74V)
    lf_125khz_radio_saadc_enable(hitag2_saadc_cb);
}

static void uninit_hitag2_hw(void) {
    lf_125khz_radio_saadc_disable();
}

/**
 * Send a single bit using correct BPLM encoding
 * 
 * BPLM (Binary Pulse Length Modulation) from Proxmark3 hitag2.c:
 * - Each bit starts with a PULSE (field OFF/dropped)
 * - Followed by field ON for variable duration
 * - Bit value encoded in the ON duration
 * 
 * Pattern: |___PULSE___|------ON------|
 * 
 * Bit 0 timing (160μs total per spec):
 *   - Pulse OFF: HITAG2_BPLM_PULSE_US (48μs = 6 Tc)
 *   - Field ON:  HITAG2_BPLM_BIT0_HIGH_US (112μs = 14 Tc)
 *   - TOTAL: 48 + 112 = 160μs = 20 Tc ✓
 * 
 * Bit 1 timing (240μs total per spec):
 *   - Pulse OFF: HITAG2_BPLM_PULSE_US (48μs = 6 Tc)
 *   - Field ON:  HITAG2_BPLM_BIT1_HIGH_US (192μs = 24 Tc)
 *   - TOTAL: 48 + 192 = 240μs = 30 Tc ✓
 * 
 * This matches Proxmark3's hitag2_reader_send_bit() exactly.
 * 
 * @param bit The bit value (0 or 1)
 */
static void hitag2_send_bit(uint8_t bit) {
    // Soft pulse LOW: reduce duty cycle to 0 (field reduces but doesn't hard-stop)
    set_lf_pwm_duty(0);
    bsp_delay_us(HITAG2_PWM_SETTLE_US);  // Let PWM ramp down cleanly
    bsp_delay_us(HITAG2_BPLM_PULSE_US - HITAG2_PWM_SETTLE_US);  // Rest of pulse time
    
    // Restore duty cycle to 2 (50% = field on)
    set_lf_pwm_duty(2);
    bsp_delay_us(HITAG2_PWM_SETTLE_US);  // Let PWM stabilize ON
    
    if (bit & 0x01) {
        // Bit 1: Longer ON time
        bsp_delay_us(HITAG2_BPLM_BIT1_HIGH_US - HITAG2_PWM_SETTLE_US);  // Rest of ON time
    } else {
        // Bit 0: Shorter ON time
        bsp_delay_us(HITAG2_BPLM_BIT0_HIGH_US - HITAG2_PWM_SETTLE_US);  // Rest of ON time
    }
}

/**
 * Send START_AUTH command (5 bits: 11000)
 * This initiates communication with tag in public mode
 * 
 * Called within timeslot for precise timing
 */
static void hitag2_send_start_auth(void) {
    uint8_t cmd = HITAG2_START_AUTH_CMD;
    
    NRF_LOG_INFO("Transmitting START_AUTH with BPLM encoding (soft pulse)...");
    
    // Send 5 bits MSB first: 1, 1, 0, 0, 0
    for (int i = 0; i < HITAG2_START_AUTH_BITS; i++) {
        uint8_t bit = (cmd >> (7 - i)) & 0x01;
        NRF_LOG_INFO("Bit %d: %d", i, bit);
        hitag2_send_bit(bit);
    }
    
    // Add soft terminating pulse to mark end of bit 5
    // This ensures tag sees all 5 bits correctly
    NRF_LOG_INFO("Adding soft terminating pulse");
    set_lf_pwm_duty(0);  // Soft pulse low
    bsp_delay_us(HITAG2_BPLM_PULSE_US);
    set_lf_pwm_duty(2);  // Restore to 50% for listening
    
    // Field now stays at 50% duty for tag response
    NRF_LOG_INFO("START_AUTH transmission complete - sent %d bits", HITAG2_START_AUTH_BITS);
}

/**
 * Edge detection layer: Convert SAADC samples to Manchester intervals
 * 
 * Manchester decoder expects time intervals between edges, but SAADC gives
 * voltage samples. This function detects threshold crossings (edges) and
 * calculates intervals between them.
 * 
 * @param samples Array of SAADC samples (0-4095, 12-bit)
 * @param sample_count Number of samples
 * @param intervals Output array for calculated intervals
 * @param max_intervals Maximum intervals to store
 * @return Number of intervals detected
 */
static int hitag2_detect_edges_from_saadc(uint16_t *samples, int sample_count,
                                           uint16_t *intervals, int max_intervals) {
    // POST-TRANSMISSION ANALYSIS: Skip muzzle flash (RFIDler strategy)
    // Skip first 1200 samples (~10ms) to eliminate:
    //   - Power-up transients (2.5ms)
    //   - START_AUTH transmission (~1ms)
    //   - TX->RX wait period (5ms)
    // SYNCHRONOUS MANCHESTER DECODER WITH PREAMBLE CLOCK RECOVERY
    // Based on expert advice: Use 11111 preamble to recover tag's clock
    // T0 = carrier period (1/125kHz), but frequency may vary - use sync to measure!
    
    int start_idx = 1200;  // Skip ~10ms muzzle flash
    
    if (sample_count < start_idx + 5000) {
        NRF_LOG_INFO("Not enough samples for preamble detection");
        return 0;
    }
    
    NRF_LOG_INFO("Synchronous Manchester decoder with preamble clock recovery");
    
    // Step 1: Apply low-pass filter y[n] = (x[n] + x[n-1]) / 2
    // This smooths noise while preserving signal structure
    uint16_t *filtered = (uint16_t *)malloc(sample_count * sizeof(uint16_t));
    if (!filtered) {
        NRF_LOG_INFO("Failed to allocate filter buffer");
        return 0;
    }
    
    filtered[start_idx] = samples[start_idx];
    for (int i = start_idx + 1; i < sample_count; i++) {
        filtered[i] = (samples[i] + samples[i-1]) / 2;
    }
    
    // Find baseline for peak detection
    uint16_t min_sample = 4095, max_sample = 0;
    for (int i = start_idx; i < start_idx + 5000; i++) {
        if (filtered[i] < min_sample) min_sample = filtered[i];
        if (filtered[i] > max_sample) max_sample = filtered[i];
    }
    uint16_t center = (min_sample + max_sample) / 2;
    
    NRF_LOG_INFO("Signal baseline: center=%d, swing=%d", center, max_sample - min_sample);
    
    // Step 2: Find 5 preamble peaks (11111 sync pattern)
    // These are falling edges 32T0 apart
    uint32_t peaks[5];
    int peak_count = 0;
    
    for (int i = start_idx + 1; i < sample_count - 1 && peak_count < 5; i++) {
        // Local maximum above baseline
        if (filtered[i] > filtered[i-1] && 
            filtered[i] > filtered[i+1] &&
            filtered[i] > center) {
            
            peaks[peak_count++] = i;
            NRF_LOG_INFO("Preamble peak %d at sample %d", peak_count, i);
            
            i += 20;  // Skip ahead to avoid double-counting
        }
    }
    
    if (peak_count < 5) {
        NRF_LOG_INFO("Preamble not found (only %d peaks detected)", peak_count);
        free(filtered);
        return 0;
    }
    
    // Step 3: Calculate τ = (fifth peak - first peak) / 4
    // This gives us the time between two edges that the tag is broadcasting
    // Tag's actual clock period (handles frequency variations!)
    uint32_t total_samples = peaks[4] - peaks[0];
    uint32_t total_time_us = total_samples * 8;  // Convert samples to µs
    uint16_t tau = total_time_us / 4;  // Time between edges (bit period)
    uint16_t half_tau = tau / 2;
    
    NRF_LOG_INFO("Clock recovery: τ=%dµs (from %d samples over 4 periods)", tau, total_samples);
    
    // Step 4: Synchronous Manchester sampling
    // Sample at first_peak + τ/2, then every τ for 32 bits
    // Compare sample at T with sample at T + τ/2 for Manchester decode
    uint32_t sample_time_us = (peaks[0] * 8) + half_tau;
    uint32_t uid = 0;
    int bits_decoded = 0;
    
    NRF_LOG_INFO("Starting synchronous sampling from %dµs", sample_time_us);
    
    for (int bit = 0; bit < 32; bit++) {
        // Sample at T (beginning of bit period)
        uint32_t t1_idx = sample_time_us / 8;
        if (t1_idx >= sample_count - 1) break;
        uint16_t val1 = filtered[t1_idx];
        
        // Sample at T + τ/2 (middle of bit period)
        uint32_t t2_idx = (sample_time_us + half_tau) / 8;
        if (t2_idx >= sample_count - 1) break;
        uint16_t val2 = filtered[t2_idx];
        
        // Manchester decode using derivative: y[n] - y[n-1]
        // If derivative < 0 (falling edge), bit = 1
        // If derivative > 0 (rising edge), bit = 0
        int16_t derivative = (int16_t)val2 - (int16_t)val1;
        
        if (derivative < 0) {
            // Falling edge = bit 1
            uid |= (1 << bit);
        }
        // else: Rising edge = bit 0 (uid bit already 0)
        
        if (bit < 8) {
            NRF_LOG_INFO("Bit %d: val1=%d val2=%d derivative=%d → %d", 
                         bit, val1, val2, derivative, (uid >> bit) & 1);
        }
        
        bits_decoded++;
        sample_time_us += tau;  // Move to next bit period
    }
    
    // Store UID in intervals array (compatibility with existing code structure)
    if (bits_decoded == 32) {
        // Store as 32-bit value in first 4 interval slots
        intervals[0] = uid & 0xFFFF;
        intervals[1] = (uid >> 16) & 0xFFFF;
        NRF_LOG_INFO("Decoded 32-bit UID: 0x%08lX", uid);
        free(filtered);
        return 32;  // Return bit count
    }
    
    NRF_LOG_INFO("Incomplete decode: only %d bits", bits_decoded);
    free(filtered);
    return 0;
}

/**
 * Timeslot callback for time-critical BPLM transmission
 * This is called within a radio timeslot for precise timing
 * 
 * Following T55xx pattern: Start field INSIDE timeslot for proper control
 */
static void hitag2_timeslot_callback(void) {
    // Start field first to power tag (following T55xx pattern)
    start_lf_125khz_radio();
    
    // Wait for tag to power up (2.5ms)
    bsp_delay_us(HITAG_T_WAIT_POWERUP_US);
    
    // Wait to be in START_AUTH window (464μs)
    bsp_delay_us(HITAG_T_WAIT_START_AUTH_US);
    
    // Send START_AUTH command with correct BPLM encoding
    // Field is ON, first bit will drop it (pulse), then bring it back
    hitag2_send_start_auth();
    
    // Field stays ON for tag to respond
    // SAADC will sample the response continuously outside timeslot
    NRF_LOG_INFO("START_AUTH transmitted, field ON for response");
}


/**
 * Universal Translator: Maps real-world µs to decoder's expected microseconds
 * 
 * The Hitag2 decoder (hitag.c) expects microsecond timing values:
 * - SHORT (Period 0): 128µs target (accepts 64-192µs)
 * - LONG (Period 2): 256µs target (accepts 192-320µs)
 * 
 * Our Paxton tags send RF/32 timing (faster than standard RF/50):
 * - Real Short: ~96-168µs (widened to capture jittery edges)
 * - Real Long: ~200-380µs (extended for slow responses)
 * 
 * This translator normalizes tag timing to standard decoder expectations.
 */
static uint8_t translate_interval(uint16_t real_us) {
    // Real Short (60-180µs) → Decoder Short (128µs)
    // REDESIGN: Fixed boundary gap (was <=185 then >185, created no-man's-land at 186-191µs)
    // Clean boundaries: 60-180 SHORT, 181-380 LONG
    if (real_us >= 60 && real_us < 181) {
        return 128;  // HITAG_T_SHORT - decoder recognizes as period 0
    }
    
    // Real Long (181-380µs) → Decoder Long (200µs)
    // REDESIGN: Start at 181 (not 186) to eliminate gap
    // Safely > 192 threshold, decoder recognizes as period 2
    if (real_us >= 181 && real_us <= 380) {
        return 200;  // Safely > 192 threshold, decoder recognizes as period 2
    }
    
    // Noise/glitches outside expected ranges
    return 0;
}

/**
 * Attempt to read Hitag2 tag UID using RTF protocol
 * 
 * Protocol flow adapted for ChameleonUltra hardware:
 * 1. Start field to power tag
 * 2. Request timeslot for time-critical BPLM transmission
 * 3. Send START_AUTH with correct OFF-then-ON BPLM pattern
 * 4. Receive Manchester response via SAADC sampling
 * 5. Detect edges from samples and feed intervals to decoder
 * 
 * Based on:
 * - Proxmark3 hitag2.c for BPLM encoding and timing
 * - ChameleonUltra T55xx for timeslot usage
 * - ChameleonUltra HID for SAADC sampling
 */
bool hitag2_read(uint8_t *data, uint32_t timeout_ms) {
    NRF_LOG_INFO("Hitag2 START_AUTH with SAADC sampling + edge detection");
    
    // Allocate codec for Manchester decoding (tag response)
    void *codec = hitag2.alloc();
    if (codec == NULL) {
        NRF_LOG_ERROR("Failed to allocate Hitag2 codec");
        return false;
    }
    hitag2.decoder.start(codec, 0);
    
    // Initialize circular buffer for SAADC samples
    cb_init(&cb, HITAG2_BUFFER_SIZE, sizeof(uint16_t));
    
    // Initialize SAADC for analog sampling (like HID)
    init_hitag2_hw();
    
    // Request timeslot for transmission
    request_timeslot(80000, hitag2_timeslot_callback);  // 80ms for full response
    
    NRF_LOG_INFO("START_AUTH transmitted, collecting SAADC samples...");
    
    // Use larger static array to hold full response
    // DOUBLED: 16384 samples = 131ms at 125kHz (was 8192 = 65ms)
    // CRITICAL: Need full buffer to capture complete 32-bit UID + CRC
    // Static to avoid stack overflow (32KB is too large for stack)
    static uint16_t samples[16384];
    int sample_count = 0;
    
    // Continuous drain loop: actively drain buffer for 80ms (matches timeslot)
    // ALIGNED: Matches hardware timeslot duration for consistent timing
    // This prevents circular buffer overflow and captures complete tag response
    autotimer *p_at = bsp_obtain_timer(0);  // Obtain timer with 0 initial value
    
    while (NO_TIMEOUT_1MS(p_at, 80) && sample_count < 16384) {
        uint16_t val;
        // Drain all available samples from circular buffer
        while (cb_pop_front(&cb, &val) && sample_count < 16384) {
            samples[sample_count++] = val;
        }
        // Brief yield to allow SAADC interrupt to fire
        bsp_delay_us(100);
    }
    
    bsp_return_timer(p_at);  // Return timer to pool
    
    NRF_LOG_INFO("Collected %d SAADC samples", sample_count);
    
    // Check if we got any samples at all
    if (sample_count == 0) {
        NRF_LOG_ERROR("No SAADC samples collected - SAADC may not be running!");
        NRF_LOG_ERROR("Check that lf_125khz_radio_saadc_enable() was called");
        stop_lf_125khz_radio();
        uninit_hitag2_hw();
        cb_free(&cb);
        hitag2.free(codec);
        return false;
    }
    
    NRF_LOG_INFO("Detecting edges from %d samples...", sample_count);
    
    // Detect edges and get intervals
    uint16_t intervals[128];
    int edge_count = hitag2_detect_edges_from_saadc(samples, sample_count, intervals, 128);
    
    if (edge_count == 0) {
        NRF_LOG_WARNING("No edges detected from samples - check signal levels");
        stop_lf_125khz_radio();
        uninit_hitag2_hw();
        cb_free(&cb);
        hitag2.free(codec);
        return false;
    }
    
    // SOF Detection: Scan for 5 consecutive short intervals (80-185µs)
    // RELAXED: Was 80-160µs, now 80-185µs to capture timing jitter
    // This identifies the 11111 SOF header with wider tolerance
    int sof_start = -1;
    for (int i = 0; i < edge_count - 5; i++) {
        int consecutive_short = 0;
        for (int j = 0; j < 5; j++) {
            if (intervals[i+j] >= 80 && intervals[i+j] <= 185) {
                consecutive_short++;
            }
        }
        if (consecutive_short >= 5) {
            sof_start = i;
            NRF_LOG_INFO("SOF detected at index %d (5+ consecutive 80-185µs intervals)", sof_start);
            break;
        }
    }
    
    // Determine start position for decoder
    int decode_start = (sof_start >= 0) ? sof_start : 0;
    if (sof_start >= 0) {
        NRF_LOG_INFO("Starting decode from SOF at index %d (%d intervals to process)", 
                    sof_start, edge_count - sof_start);
        
        // DIAGNOSTIC: Log first 10 intervals with translations
        NRF_LOG_INFO("First intervals (raw → translated):");
        for (int i = sof_start; i < edge_count && i < sof_start + 10; i++) {
            uint8_t translated = translate_interval(intervals[i]);
            NRF_LOG_INFO("  INT[%d]: %dµs → %d", i - sof_start, intervals[i], translated);
        }
    } else {
        NRF_LOG_WARNING("No SOF detected - starting from beginning (may fail)");
        NRF_LOG_INFO("Feeding all %d intervals to decoder", edge_count);
    }
    
    // Feed intervals to Manchester decoder with sliding window retry
    // Use Universal Translator to convert real µs to decoder's magic numbers
    // Try offsets 0, 1, 2, 3 from SOF to handle extra noise edges
    bool ok = false;
    
    // Sliding window: Try 4 different starting positions
    for (int offset = 0; offset < 4 && !ok && decode_start + offset < edge_count; offset++) {
        int start_pos = decode_start + offset;
        NRF_LOG_INFO("Trying offset %d: decode from index %d", offset, start_pos);
        
        hitag2.decoder.start(codec, 0);  // Reset decoder state
        for (int i = start_pos; i < edge_count; i++) {
            uint16_t raw_interval = intervals[i];
            
            // Translate real-world µs to decoder's expected values (48, 112)
            // This bridges the gap between RF/32 (Paxton) and RF/50 (decoder expects)
            uint8_t translated_interval = translate_interval(raw_interval);
            
            // Feed translated interval to decoder
            if (hitag2.decoder.feed(codec, translated_interval)) {
                memcpy(data, hitag2.get_data(codec), hitag2.data_size);
                ok = true;
                NRF_LOG_INFO("Offset %d SUCCESS! Hitag2 UID: %02X%02X%02X%02X", 
                            offset, data[0], data[1], data[2], data[3]);
                break;
            }
        }
        
        if (!ok && offset < 3) {
            NRF_LOG_INFO("Offset %d failed, trying next offset...", offset);
        }
    }
    
    stop_lf_125khz_radio();
    uninit_hitag2_hw();
    cb_free(&cb);
    hitag2.free(codec);
    
    if (!ok) {
        NRF_LOG_INFO("Hitag2 tag not found - %d edges detected but decode failed", edge_count);
        NRF_LOG_INFO("Try adjusting tag position or check Manchester thresholds");
    }
    
    return ok;
}
