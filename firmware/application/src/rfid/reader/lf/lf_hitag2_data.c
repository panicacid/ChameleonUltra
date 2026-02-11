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
 * Helper: Calculate variance for quiet zone detection
 */
static uint32_t calculate_variance(uint16_t *samples, int start, int window_size) {
    uint32_t mean = 0;
    for (int i = 0; i < window_size; i++) {
        mean += samples[start + i];
    }
    mean /= window_size;
    
    uint32_t variance = 0;
    for (int i = 0; i < window_size; i++) {
        int32_t diff = (int32_t)samples[start + i] - (int32_t)mean;
        variance += diff * diff;
    }
    return variance / window_size;
}

/**
 * Synchronous Phase-Sampling Decoder for Hitag2
 * 
 * Mathematical approach using preamble-based clock recovery:
 * 1. Find quiet zone (variance-based)
 * 2. Find 5 preamble peaks (11111 sync)
 * 3. Calculate τ = (P₅ - P₁) / 4 (bit period)
 * 4. Start sampling at P₁ + 5.5×τ (data alignment)
 * 5. Sample 32 bits using phase comparison
 * 
 * Returns: true if UID decoded, false otherwise
 */
static bool hitag2_sync_decode(uint16_t *samples, int sample_count, uint8_t *data) {
    int start_idx = 200;  // Skip only initial TX noise
    
    if (sample_count < 5000) {
        NRF_LOG_ERROR("Not enough samples for sync decode");
        return false;
    }
    
    NRF_LOG_INFO("Synchronous Phase-Sampling decoder with scanning loop");
    
    // Step 1: Apply 4-sample moving average to nullify 125kHz carrier
    // This averages 4 samples (32µs) to cancel out carrier oscillations
    uint16_t *filtered = (uint16_t *)malloc(sample_count * sizeof(uint16_t));
    if (!filtered) {
        NRF_LOG_ERROR("Failed to allocate filter buffer");
        return false;
    }
    
    // Initialize first 3 samples
    for (int i = start_idx; i < start_idx + 3 && i < sample_count; i++) {
        filtered[i] = samples[i];
    }
    
    // 4-sample moving average: filtered[i] = (x[i] + x[i-1] + x[i-2] + x[i-3]) / 4
    for (int i = start_idx + 3; i < sample_count; i++) {
        filtered[i] = (samples[i] + samples[i-1] + samples[i-2] + samples[i-3]) / 4;
    }
    
    NRF_LOG_INFO("Applied 4-sample carrier-nulling filter");
    
    // SCANNING LOOP: Search entire buffer for valid tag signal
    // Don't give up on first invalid candidate - could be ghost noise
    int search_start = start_idx;
    int attempt = 0;
    
    while (search_start < sample_count - 5000) {
        attempt++;
        NRF_LOG_INFO("=== Scan attempt %d starting at sample %d ===", attempt, search_start);
        
        // Step 2: Find quiet zone (low variance = stable carrier)
        #define QUIET_WINDOW 100
        #define QUIET_THRESHOLD 200
        
        int quiet_zone_end = search_start;
        for (int i = search_start; i < sample_count - QUIET_WINDOW; i += 50) {
            uint32_t variance = calculate_variance(filtered, i, QUIET_WINDOW);
            if (variance < QUIET_THRESHOLD) {
                quiet_zone_end = i + QUIET_WINDOW;
                NRF_LOG_INFO("Quiet zone ending at sample %d (~%dms)", 
                            quiet_zone_end, (quiet_zone_end * 8) / 1000);
                break;
            }
        }
        
        // Step 3: Find modulation start (high variance = tag responding)
        int preamble_start = quiet_zone_end;
        bool modulation_found = false;
        for (int i = quiet_zone_end; i < sample_count - QUIET_WINDOW; i += 10) {
            uint32_t variance = calculate_variance(filtered, i, QUIET_WINDOW);
            if (variance > QUIET_THRESHOLD * 3) {
                preamble_start = i;
                NRF_LOG_INFO("Modulation start at sample %d (~%dms)", 
                            preamble_start, (preamble_start * 8) / 1000);
                modulation_found = true;
                break;
            }
        }
        
        if (!modulation_found) {
            NRF_LOG_WARNING("No modulation found from sample %d, search exhausted", search_start);
            break;  // End of buffer reached
        }
        
        // Step 4: Find 5 preamble peaks (11111 sync pattern)
        uint16_t global_min = 4095, global_max = 0;
        for (int i = preamble_start; i < sample_count && i < preamble_start + 2000; i++) {
            if (filtered[i] < global_min) global_min = filtered[i];
            if (filtered[i] > global_max) global_max = filtered[i];
        }
        uint16_t center = (global_min + global_max) / 2;
        
        uint32_t peaks[5];
        uint16_t peak_heights[5];  // Track peak heights for quality check
        int peak_count = 0;
        
        for (int i = preamble_start + 1; i < sample_count - 1 && peak_count < 5; i++) {
            // Local maximum with 150 ADC hysteresis to filter noise
            // Peak must be 150 ADC above baseline to prevent carrier ripple detection
            if (filtered[i] > filtered[i-1] && 
                filtered[i] > filtered[i+1] &&
                filtered[i] > center + 150) {  // Hysteresis: 150 ADC above baseline
                
                peaks[peak_count] = i;
                peak_heights[peak_count] = filtered[i] - center;  // Store height
                NRF_LOG_INFO("Preamble peak %d at sample %d (height=%d above center)", 
                            peak_count + 1, i, peak_heights[peak_count]);
                peak_count++;
                i += 20;  // Skip to avoid double-counting
            }
        }
        
        if (peak_count < 5) {
            NRF_LOG_WARNING("Only found %d peaks, need 5 - skipping to next candidate", peak_count);
            search_start = preamble_start + 500;  // Skip past this noise event
            continue;  // Try next location
        }
        
        // Preamble quality check: Reject glitches with inconsistent peak heights
        uint16_t min_height = 65535, max_height = 0;
        for (int i = 0; i < 5; i++) {
            if (peak_heights[i] < min_height) min_height = peak_heights[i];
            if (peak_heights[i] > max_height) max_height = peak_heights[i];
        }
        
        if (min_height < (max_height / 3)) {
            NRF_LOG_WARNING("Preamble quality check failed: min=%d max=%d (glitch/noise burst)",
                           min_height, max_height);
            search_start = preamble_start + 500;
            continue;  // Reject this candidate
        }
        
        // Step 5: Calculate τ (bit period) from preamble with snap-to-grid
        // τ = (P₅ - P₁) / 4 (5 peaks = 4 bit periods)
        uint32_t raw_tau = (peaks[4] - peaks[0]) / 4;
        uint32_t tau_samples;
        
        // Snap-to-grid clock recovery for carrier-synchronous tags
        // Hitag2 is RF/32 (256µs) or RF/40 (320µs)
        if (raw_tau >= 29 && raw_tau <= 35) {
            tau_samples = 32;  // Lock to RF/32
            NRF_LOG_INFO("Snap-to-grid: raw τ=%d → locked to 32 samples (256µs, RF/32)", raw_tau);
        } else if (raw_tau >= 37 && raw_tau <= 43) {
            tau_samples = 40;  // Lock to RF/40
            NRF_LOG_INFO("Snap-to-grid: raw τ=%d → locked to 40 samples (320µs, RF/40)", raw_tau);
        } else {
            tau_samples = raw_tau;
            NRF_LOG_INFO("Clock recovery: τ=%d samples (%dµs) - no snap (outside grid range)", 
                        tau_samples, tau_samples * 8);
        }
        
        uint32_t tau_us = tau_samples * 8;
        
        // STRICT VALIDATION: Minimum τ = 30 samples (240µs)
        // This rejects ghost noise (τ=25) and ensures we find real tag (τ=37)
        if (tau_samples < 30 || tau_samples > 56) {
            NRF_LOG_WARNING("τ out of range: %d samples (%dµs) - expect 30-56 samples (240-448µs)", 
                         tau_samples, tau_us);
            NRF_LOG_WARNING("Likely ghost noise - continuing scan for real tag");
            search_start = preamble_start + 500;  // Skip past this noise event
            continue;  // Try next location
        }
        
        NRF_LOG_INFO("Clock recovery validated: τ=%d samples (%dµs) ✓", tau_samples, tau_us);
        
        // Step 6: Calculate data start (5.0 bit periods after first peak)
        // Center-to-center alignment: P1 is at phase 0.25 (HIGH center)
        // 5 complete periods later = phase 0.25 of first data bit (HIGH center)
        uint32_t data_start_sample = peaks[0] + (tau_samples * 5);  // 5.0 × τ
        
        NRF_LOG_INFO("Data start at sample %d (P1=%d + 5.0×τ)", 
                    data_start_sample, peaks[0]);
        
        if (data_start_sample + (32 * tau_samples) >= sample_count) {
            NRF_LOG_WARNING("Data extends beyond buffer - skipping to next candidate");
            search_start = preamble_start + 500;
            continue;  // Try next location
        }
        
        // Step 7: Phase-based sampling (32 bits)
        uint32_t uid = 0;
        bool decode_success = true;
        uint32_t derivative_sum = 0;  // Track signal strength
        
        for (int bit = 0; bit < 32; bit++) {
            uint32_t t_start = data_start_sample + (bit * tau_samples);
            uint32_t t_mid = t_start + (tau_samples / 2);
            
            if (t_mid >= sample_count) {
                NRF_LOG_WARNING("Bit %d sampling beyond buffer", bit);
                decode_success = false;
                break;
            }
            
            uint16_t v_start = filtered[t_start];
            uint16_t v_mid = filtered[t_mid];
            int16_t derivative = (int16_t)v_mid - (int16_t)v_start;
            
            // Track signal strength for first 8 bits
            if (bit < 8) {
                derivative_sum += (derivative < 0) ? -derivative : derivative;
            }
            
            // Derivative-based edge detection: v_mid < v_start = falling = 1
            if (v_mid < v_start) {
                uid |= (1 << bit);
            }
            
            // Debug logging for first few bits
            if (bit < 8) {
                NRF_LOG_INFO("Bit %d: V_start=%d V_mid=%d derivative=%d → %d", 
                            bit, v_start, v_mid, derivative, (v_mid < v_start) ? 1 : 0);
            }
        }
        
        if (!decode_success) {
            search_start = preamble_start + 500;
            continue;  // Try next location
        }
        
        // Step 8: Signal strength check - reject weak ghost noise
        uint32_t avg_derivative = derivative_sum / 8;
        if (avg_derivative < 100) {
            NRF_LOG_WARNING("Signal too weak (Ghost): avg derivative=%d ADC", avg_derivative);
            NRF_LOG_WARNING("Rejecting weak signal - continuing scan for real tag");
            search_start = preamble_start + 500;
            continue;  // Skip weak signal, keep scanning
        }
        
        // SUCCESS! Found valid tag and decoded UID
        NRF_LOG_INFO("✓ VALID TAG FOUND after %d scan attempts", attempt);
        NRF_LOG_INFO("Decoded 32-bit UID: 0x%08X", uid);
        
        // Copy UID to output
        memcpy(data, &uid, 4);
        
        free(filtered);
        return true;
        
    }  // End of scanning loop
    
    // Exhausted buffer without finding valid tag
    NRF_LOG_ERROR("Scanned entire buffer (%d attempts) - no valid tag found", attempt);
    free(filtered);
    return false;
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
    NRF_LOG_INFO("Hitag2 START_AUTH with Synchronous Phase-Sampling decoder");
    
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
    
    // Call synchronous phase-sampling decoder
    bool success = hitag2_sync_decode(samples, sample_count, data);
    
    // Cleanup
    stop_lf_125khz_radio();
    uninit_hitag2_hw();
    cb_free(&cb);
    
    if (success) {
        NRF_LOG_INFO("SUCCESS! Hitag2 UID: %02X%02X%02X%02X", 
                    data[0], data[1], data[2], data[3]);
    } else {
        NRF_LOG_ERROR("Synchronous decoding failed");
    }
    
    return success;
}
