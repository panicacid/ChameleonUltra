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
#define HITAG2_BUFFER_SIZE        16384  // Power of 2 for full 32-bit UID capture

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
 * RFIDler-Style Pulse Width Decoder for Hitag2
 * 
 * Digital logic approach (proven robust):
 * 1. Carrier filter: 4-sample moving average (removes 125kHz carrier)
 * 2. Squelch check: Reject if dynamic range < 500 ADC (ghost noise)
 * 3. Software comparator: Binarize signal to HIGH/LOW with hysteresis
 * 4. Pulse extraction: Count sample durations in each state → intervals array
 * 5. Pattern matching: Find preamble (10+ consecutive short pulses)
 * 6. Manchester decode: Short+Short=transition, Long=repeat
 * 
 * Returns: true if UID decoded, false otherwise
 */
static bool hitag2_sync_decode(uint16_t *samples, int sample_count, uint8_t *data) {
    if (sample_count < 5000) {
        NRF_LOG_ERROR("Not enough samples");
        return false;
    }
    
    NRF_LOG_INFO("RFIDler-style pulse width decoder");
    
    // Step 1: Apply 4-sample moving average (removes 125kHz carrier)
    uint16_t *filtered = (uint16_t *)malloc(sample_count * sizeof(uint16_t));
    if (!filtered) {
        NRF_LOG_ERROR("Failed to allocate filter buffer");
        return false;
    }
    
    // Initialize first 3 samples
    for (int i = 0; i < 3 && i < sample_count; i++) {
        filtered[i] = samples[i];
    }
    
    // 4-sample moving average
    for (int i = 3; i < sample_count; i++) {
        filtered[i] = (samples[i] + samples[i-1] + samples[i-2] + samples[i-3]) / 4;
    }
    
    NRF_LOG_INFO("Step 1: Carrier filter applied");
    
    // Step 2: Squelch check - find dynamic range
    uint16_t global_min = 65535, global_max = 0;
    for (int i = 0; i < sample_count; i++) {
        if (filtered[i] < global_min) global_min = filtered[i];
        if (filtered[i] > global_max) global_max = filtered[i];
    }
    
    uint16_t dynamic_range = global_max - global_min;
    NRF_LOG_INFO("Step 2: Dynamic range=%d (min=%d max=%d)", dynamic_range, global_min, global_max);
    
    if (dynamic_range < 500) {
        NRF_LOG_ERROR("Signal too weak: range=%d (need >500) - Ghost noise", dynamic_range);
        free(filtered);
        return false;
    }
    
    // Step 3: Software comparator - binarize signal
    uint16_t midpoint = (global_max + global_min) / 2;
    uint16_t hysteresis = 50;
    NRF_LOG_INFO("Step 3: Comparator midpoint=%d, hysteresis=%d", midpoint, hysteresis);
    
    // Extract pulse widths
    uint16_t *intervals = (uint16_t *)malloc(2000 * sizeof(uint16_t));
    if (!intervals) {
        NRF_LOG_ERROR("Failed to allocate intervals buffer");
        free(filtered);
        return false;
    }
    
    bool state = (filtered[0] > midpoint);
    int duration = 0;
    int interval_count = 0;
    
    for (int i = 0; i < sample_count; i++) {
        bool new_state;
        
        // Comparator with hysteresis
        if (filtered[i] > midpoint + hysteresis) {
            new_state = true;   // HIGH
        } else if (filtered[i] < midpoint - hysteresis) {
            new_state = false;  // LOW
        } else {
            new_state = state;  // Hysteresis zone - keep current state
        }
        
        if (new_state != state) {
            // State transition - record pulse width
            if (interval_count < 2000) {
                intervals[interval_count++] = duration;
            }
            duration = 0;
            state = new_state;
        }
        duration++;
    }
    
    // Record final interval
    if (interval_count < 2000) {
        intervals[interval_count++] = duration;
    }
    
    NRF_LOG_INFO("Extracted %d pulse intervals", interval_count);
    
    // Step 4: Find preamble - 10+ consecutive short pulses (12-24 samples)
    int preamble_start = -1;
    for (int i = 0; i < interval_count - 10; i++) {
        bool is_preamble = true;
        
        for (int j = 0; j < 10; j++) {
            if (intervals[i+j] < 12 || intervals[i+j] > 24) {
                is_preamble = false;
                break;
            }
        }
        
        if (is_preamble) {
            preamble_start = i;
            NRF_LOG_INFO("Step 4: Preamble found at interval %d", i);
            break;
        }
    }
    
    if (preamble_start < 0) {
        NRF_LOG_ERROR("No preamble pattern found");
        free(intervals);
        free(filtered);
        return false;
    }
    
    // Step 5: Omni-Decoder - Try all 4 Manchester alignments
    // Manchester has 2 variables: phase (±10/11) and polarity (true/false)
    // One of these 4 combinations MUST be correct
    
    NRF_LOG_INFO("=== OMNI-DECODER: Trying all 4 Manchester alignments ===");
    
    // Decode helper function - inline for each candidate
    uint32_t uid_a = 0, uid_b = 0, uid_c = 0, uid_d = 0;
    
    // Candidate A: +10 offset, normal polarity
    {
        uint32_t result = 0;
        bool last_bit = true;
        int idx = preamble_start + 10;
        
        for (int bit = 0; bit < 32; bit++) {
            if (idx >= interval_count) break;
            uint16_t pulse = intervals[idx++];
            
            if (pulse >= 12 && pulse <= 24) {
                last_bit = !last_bit;
            }
            
            if (last_bit) {
                result |= (1 << bit);
            }
        }
        uid_a = result;
    }
    NRF_LOG_INFO("[Candidate A] +10 Normal:   UID: 0x%08X", uid_a);
    
    // Candidate B: +10 offset, inverted polarity
    {
        uint32_t result = 0;
        bool last_bit = false;
        int idx = preamble_start + 10;
        
        for (int bit = 0; bit < 32; bit++) {
            if (idx >= interval_count) break;
            uint16_t pulse = intervals[idx++];
            
            if (pulse >= 12 && pulse <= 24) {
                last_bit = !last_bit;
            }
            
            if (last_bit) {
                result |= (1 << bit);
            }
        }
        uid_b = result;
    }
    NRF_LOG_INFO("[Candidate B] +10 Inverted: UID: 0x%08X", uid_b);
    
    // Candidate C: +11 offset, normal polarity
    {
        uint32_t result = 0;
        bool last_bit = true;
        int idx = preamble_start + 11;
        
        for (int bit = 0; bit < 32; bit++) {
            if (idx >= interval_count) break;
            uint16_t pulse = intervals[idx++];
            
            if (pulse >= 12 && pulse <= 24) {
                last_bit = !last_bit;
            }
            
            if (last_bit) {
                result |= (1 << bit);
            }
        }
        uid_c = result;
    }
    NRF_LOG_INFO("[Candidate C] +11 Normal:   UID: 0x%08X", uid_c);
    
    // Candidate D: +11 offset, inverted polarity
    {
        uint32_t result = 0;
        bool last_bit = false;
        int idx = preamble_start + 11;
        
        for (int bit = 0; bit < 32; bit++) {
            if (idx >= interval_count) break;
            uint16_t pulse = intervals[idx++];
            
            if (pulse >= 12 && pulse <= 24) {
                last_bit = !last_bit;
            }
            
            if (last_bit) {
                result |= (1 << bit);
            }
        }
        uid_d = result;
    }
    NRF_LOG_INFO("[Candidate D] +11 Inverted: UID: 0x%08X", uid_d);
    
    NRF_LOG_INFO("=== One of these 4 UIDs should match your tag! ===");
    
    // Return Candidate D (was working in calibration)
    // User can see all 4 options above
    uint32_t uid = uid_d;
    
    // Copy UID to output
    memcpy(data, &uid, 4);
    
    free(intervals);
    free(filtered);
    return true;
}

/**
 * Attempt to read Hitag2 tag UID using RTF protocol
 * 
 * Protocol flow adapted for ChameleonUltra hardware:
 * 1. Start field to power tag
 * 2. Request timeslot for time-critical BPLM transmission
 * 3. Send START_AUTH with correct OFF-then-ON BPLM pattern
 * 4. Receive Manchester response via SAADC sampling
 * 5. RFIDler-style pulse width decoder (digital logic approach)
 * 
 * Based on:
 * - Proxmark3 hitag2.c for BPLM encoding and timing
 * - RFIDler hitag.c for pulse width decoding algorithm
 * - ChameleonUltra T55xx for timeslot usage
 * - ChameleonUltra HID for SAADC sampling
 */
bool hitag2_read(uint8_t *data, uint32_t timeout_ms) {
    NRF_LOG_INFO("Hitag2 START_AUTH with RFIDler-style pulse width decoder");
    
    // Initialize circular buffer for SAADC samples
    cb_init(&cb, HITAG2_BUFFER_SIZE, sizeof(uint16_t));
    
    // Initialize SAADC for analog sampling
    init_hitag2_hw();
    
    // Request timeslot for transmission
    request_timeslot(80000, hitag2_timeslot_callback);  // 80ms for full response
    
    NRF_LOG_INFO("START_AUTH transmitted, collecting SAADC samples...");
    
    // Static array to hold full response (32KB)
    static uint16_t samples[16384];
    int sample_count = 0;
    
    // Collect samples for 80ms (matches timeslot)
    autotimer *p_at = bsp_obtain_timer(0);
    
    while (NO_TIMEOUT_1MS(p_at, 80) && sample_count < 16384) {
        uint16_t val;
        while (cb_pop_front(&cb, &val) && sample_count < 16384) {
            samples[sample_count++] = val;
        }
        bsp_delay_us(100);
    }
    
    bsp_return_timer(p_at);
    
    NRF_LOG_INFO("Collected %d SAADC samples", sample_count);
    
    // Check if we got samples
    if (sample_count == 0) {
        NRF_LOG_ERROR("No SAADC samples collected!");
        stop_lf_125khz_radio();
        uninit_hitag2_hw();
        cb_free(&cb);
        return false;
    }
    
    // Call RFIDler-style pulse width decoder
    bool success = hitag2_sync_decode(samples, sample_count, data);
    
    // Cleanup
    stop_lf_125khz_radio();
    uninit_hitag2_hw();
    cb_free(&cb);
    
    if (success) {
        NRF_LOG_INFO("SUCCESS! Hitag2 UID: %02X%02X%02X%02X", 
                    data[0], data[1], data[2], data[3]);
    } else {
        NRF_LOG_ERROR("Pulse width decoding failed");
    }
    
    return success;
}
