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
// 50ms collection = 6,250 samples needed
// Use 8192 (power of 2) for 30% safety margin
#define HITAG2_BUFFER_SIZE        8192  // Was 128 - CRITICAL FIX for buffer overflow

// Calibrated threshold based on scope measurements
// Scope data: Floor=0V, Weak data peaks=2.25V, Strong peaks=3.28V
// Option 1: Midpoint between floor and weak peaks = (0V + 2.25V) / 2 = 1.125V
// ADC value (12-bit, 3.3V ref): (1.125 / 3.3) × 4095 = 1395 ADC
// Margin above threshold: 2793 - 1395 = 1398 ADC (1.125V) - Much safer!
#define HITAG2_ADC_THRESHOLD_CALIBRATED 1395  // 1.125V for reliable 2.25V weak peak detection

static circular_buffer cb;

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
    // Start with PULSE (field OFF) - this is the BPLM signature
    stop_lf_125khz_radio();
    bsp_delay_us(HITAG2_PWM_SETTLE_US);  // Let PWM ramp down cleanly
    bsp_delay_us(HITAG2_BPLM_PULSE_US - HITAG2_PWM_SETTLE_US);  // Rest of pulse time
    
    // Then field ON for duration that encodes the bit value
    start_lf_125khz_radio();
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
    
    NRF_LOG_INFO("Transmitting START_AUTH with BPLM encoding...");
    
    // Send 5 bits MSB first: 1, 1, 0, 0, 0
    // Bit positions in 0xC0 (11000000):
    // Bit 7: 1, Bit 6: 1, Bit 5: 0, Bit 4: 0, Bit 3: 0
    // 
    // CRITICAL: Use explicit loop counter to ensure exactly 5 bits are sent
    // PicoScope confirmed previous loop only sent 4 bits!
    for (int i = 0; i < HITAG2_START_AUTH_BITS; i++) {
        uint8_t bit = (cmd >> (7 - i)) & 0x01;
        NRF_LOG_INFO("Bit %d: %d", i, bit);
        hitag2_send_bit(bit);
    }
    
    // CRITICAL: Add terminating pulse to mark end of bit 5
    // Each BPLM bit is defined by [falling edge] → OFF → ON → [next falling edge]
    // Bit 5 needs a terminating falling edge to mark its end
    // Without this, scope only sees 4 complete bits (1100 instead of 11000)
    NRF_LOG_INFO("Adding terminating pulse to mark end of bit 5");
    stop_lf_125khz_radio();
    bsp_delay_us(HITAG2_BPLM_PULSE_US);  // 69µs pulse (same as bits)
    start_lf_125khz_radio();
    
    // Field is now ON for tag to receive power and respond
    NRF_LOG_INFO("Field ON for tag response");
    
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
    // Find min/max for diagnostic purposes
    uint16_t min_sample = 4095, max_sample = 0;
    for (int i = 0; i < sample_count; i++) {
        if (samples[i] < min_sample) min_sample = samples[i];
        if (samples[i] > max_sample) max_sample = samples[i];
    }
    
    // Use calibrated threshold based on scope measurements
    // Scope showed weak data peaks at 2.25V that need reliable detection
    // Calibrated to 1.125V (midpoint between 0V floor and 2.25V weak peaks)
    uint16_t threshold = HITAG2_ADC_THRESHOLD_CALIBRATED;  // 1395 ADC = 1.125V
    
    NRF_LOG_INFO("Sample range: min=%d (~%dmV), max=%d (~%dmV)",
                 min_sample, (min_sample * 3300) / 4095,
                 max_sample, (max_sample * 3300) / 4095);
    NRF_LOG_INFO("Using CALIBRATED threshold: %d ADC (~%dmV) for 2.25V weak peak detection",
                 threshold, (threshold * 3300) / 4095);
    
    int16_t last_sample = -1;
    int last_edge_index = 0;
    int interval_count = 0;
    
    for (int i = 0; i < sample_count && interval_count < max_intervals; i++) {
        if (last_sample >= 0) {
            // Detect threshold crossing (edge)
            bool edge_detected = false;
            if (last_sample < threshold && samples[i] >= threshold) {
                // Rising edge (LOW → HIGH)
                edge_detected = true;
            } else if (last_sample >= threshold && samples[i] < threshold) {
                // Falling edge (HIGH → LOW)
                edge_detected = true;
            }
            
            if (edge_detected) {
                // Calculate interval in sample units
                int sample_interval = i - last_edge_index;
                
                // CRITICAL: Convert to microseconds for decoder
                // SAADC runs at 125kHz (PWM 500kHz/4) = 8µs per sample
                // Decoder expects µs: T_LOW=48µs (0x30), T_HIGH=112µs (0x70)
                // Hitag2 short pulse ~125µs = ~15-16 samples = 120-128µs
                const uint8_t MICROSECONDS_PER_SAMPLE = 8;
                uint16_t interval_us = sample_interval * MICROSECONDS_PER_SAMPLE;
                
                // FIXED: Remove 255µs cap - use full uint16_t range
                // FIXED: Filter noise - skip intervals <40µs (glitches)
                // FIXED: Skip field stabilization - first edge if >1000µs
                if (interval_us < 40) {
                    // Skip noise/glitches
                    last_edge_index = i;
                } else if (interval_count == 0 && interval_us > 1000) {
                    // Skip field stabilization (first long edge)
                    last_edge_index = i;
                } else {
                    // Store full interval value (no cap)
                    intervals[interval_count++] = interval_us;
                    last_edge_index = i;
                    
                    if (interval_count <= 10) {
                        NRF_LOG_INFO("Edge %d: %d samples = %dµs", 
                                     interval_count, sample_interval, interval_us);
                    }
                }
            }
        }
        last_sample = samples[i];
    }
    
    NRF_LOG_INFO("Detected %d edges from %d samples", interval_count, sample_count);
    return interval_count;
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
    request_timeslot(15000, hitag2_timeslot_callback);
    
    NRF_LOG_INFO("START_AUTH transmitted, collecting SAADC samples...");
    
    // Use larger static array to hold full response (8192 samples = 65ms at 125kHz)
    // Static to avoid stack overflow (16KB is too large for stack)
    static uint16_t samples[8192];
    int sample_count = 0;
    
    // Continuous drain loop: don't sleep, actively drain buffer for 50ms
    // This prevents circular buffer overflow and captures complete tag response
    autotimer *p_at = bsp_obtain_timer(0);  // Obtain timer with 0 initial value
    
    while (NO_TIMEOUT_1MS(p_at, 50) && sample_count < 8192) {
        uint16_t val;
        // Drain all available samples from circular buffer
        while (cb_pop_front(&cb, &val) && sample_count < 8192) {
            samples[sample_count++] = val;
            
            // Log first 20 samples for debugging
            if (sample_count <= 20) {
                // Convert to millivolts: (sample / 4095) * 3300
                uint32_t mv = (val * 3300) / 4095;
                NRF_LOG_INFO("Sample[%d]: %d (~%dmV)", sample_count-1, val, mv);
            }
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
    
    // DEBUG: Log first 32 intervals in microseconds for analysis
    NRF_LOG_INFO("First 32 intervals (µs):");
    for (int i = 0; i < edge_count && i < 32; i++) {
        NRF_LOG_INFO("INT[%d]: %dµs", i, intervals[i]);
    }
    
    // Feed ALL intervals to Manchester decoder (SOF skip disabled for Paxton debugging)
    // The decoder should handle SOF internally, or we'll see F8... pattern if present
    NRF_LOG_INFO("Feeding all %d intervals to decoder (SOF skip disabled)", edge_count);
    
    bool ok = false;
    for (int i = 0; i < edge_count; i++) {
        if (hitag2.decoder.feed(codec, intervals[i])) {
            memcpy(data, hitag2.get_data(codec), hitag2.data_size);
            ok = true;
            NRF_LOG_INFO("SUCCESS! Hitag2 UID: %02X%02X%02X%02X", 
                        data[0], data[1], data[2], data[3]);
            break;
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
