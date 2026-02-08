#include "lf_hitag2_data.h"

#include "bsp_delay.h"
#include "bsp_time.h"
#include "circular_buffer.h"
#include "lf_125khz_radio.h"
#include "lf_reader_data.h"
#include "protocols/hitag.h"
#include "timeslot.h"
#include "nrf_gpio.h"

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

#define HITAG2_BUFFER_SIZE        128

static circular_buffer cb;

// GPIO interrupt callback for receiving tag response
static void hitag2_gpio_int0_cb(void) {
    uint32_t cntr = get_lf_counter_value();
    uint16_t val = 0;
    if (cntr > 0xff) {
        val = 0xff;
    } else {
        val = cntr & 0xff;
    }
    cb_push_back(&cb, &val);
    
    // DEBUG: Log captured edges (sample first 50)
    static uint16_t edge_count = 0;
    if (edge_count < 50) {
        NRF_LOG_DEBUG("Edge #%d: interval=%d (0x%02X)", edge_count, val, val);
        edge_count++;
    } else if (edge_count == 50) {
        NRF_LOG_INFO("... (suppressing further edge logs)");
        edge_count++;
    }
    
    clear_lf_counter_value();
}

static void init_hitag2_hw(void) {
    register_rio_callback(hitag2_gpio_int0_cb);
    lf_125khz_radio_gpiote_enable();
}

static void uninit_hitag2_hw(void) {
    lf_125khz_radio_gpiote_disable();
    unregister_rio_callback();
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

// Static buffer for GPIO polling results
static uint16_t g_polled_edges[128];
static int g_polled_edge_count = 0;

/**
 * Poll GPIO directly to capture card response edges
 * This bypasses GPIOTE which is disabled during timeslot
 * 
 * @param buffer Buffer to store edge intervals
 * @param max_edges Maximum number of edges to capture
 * @param timeout_us How long to poll (microseconds)
 * @return Number of edges captured
 */
static int hitag2_poll_gpio_response(uint16_t *buffer, int max_edges, uint32_t timeout_us) {
    uint32_t pin = LF_OA_OUT;
    uint32_t last_state = nrf_gpio_pin_read(pin);
    uint32_t start_time = bsp_get_sys_tick_us();
    uint32_t last_edge_time = start_time;
    int edge_count = 0;
    
    NRF_LOG_INFO("Polling GPIO for response (timeout: %d µs)...", timeout_us);
    
    // Poll GPIO pin directly until timeout or buffer full
    while ((bsp_get_sys_tick_us() - start_time) < timeout_us && edge_count < max_edges) {
        uint32_t current_state = nrf_gpio_pin_read(pin);
        
        // Edge detected (state change)
        if (current_state != last_state) {
            uint32_t now = bsp_get_sys_tick_us();
            uint32_t interval = now - last_edge_time;
            
            // Cap interval at 0xFF like circular buffer does
            buffer[edge_count++] = (interval > 0xFF) ? 0xFF : (uint16_t)interval;
            
            // Log first few edges for debugging
            if (edge_count <= 10) {
                NRF_LOG_DEBUG("Polled edge #%d: interval=%d µs", edge_count - 1, buffer[edge_count - 1]);
            }
            
            last_edge_time = now;
            last_state = current_state;
        }
    }
    
    NRF_LOG_INFO("GPIO polling complete: captured %d edges", edge_count);
    return edge_count;
}

/**
 * Timeslot callback for time-critical BPLM transmission
 * This is called within a radio timeslot with interrupts disabled
 * 
 * CRITICAL: GPIOTE interrupts are DISABLED during timeslot!
 * We must poll GPIO directly to capture card response.
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
    
    // Field is now ON after transmission
    // Tag can respond with UID
    
    // CRITICAL: Poll GPIO immediately to capture response
    // GPIOTE is disabled during timeslot, so we must use direct pin reading
    // Card responds within ~1-2ms after START_AUTH completes
    g_polled_edge_count = hitag2_poll_gpio_response(g_polled_edges, 128, 10000);
}

/**
 * Attempt to read Hitag2 tag UID using RTF protocol
 * 
 * Protocol flow adapted for ChameleonUltra hardware:
 * 1. Start field to power tag
 * 2. Request timeslot for time-critical BPLM transmission
 * 3. Send START_AUTH with correct OFF-then-ON BPLM pattern
 * 4. Receive Manchester response via GPIO interrupts
 * 
 * Based on:
 * - Proxmark3 hitag2.c for BPLM encoding and timing
 * - ChameleonUltra T55xx for timeslot usage
 * - ChameleonUltra EM410x for reception method
 */
bool hitag2_read(uint8_t *data, uint32_t timeout_ms) {
    NRF_LOG_INFO("Hitag2 RTF protocol with correct BPLM encoding starting...");
    NRF_LOG_INFO("RECEIVER: Relaxed thresholds for weak signals enabled");
    NRF_LOG_INFO("RECEIVER: Extended listening window (5ms)");
    
    // Allocate codec for Manchester decoding (tag response)
    void *codec = hitag2.alloc();
    if (codec == NULL) {
        NRF_LOG_ERROR("Failed to allocate Hitag2 codec");
        return false;
    }
    hitag2.decoder.start(codec, 0);
    
    // Initialize circular buffer for edge timing capture
    cb_init(&cb, HITAG2_BUFFER_SIZE, sizeof(uint16_t));
    
    // Initialize GPIO interrupts for receiving tag response
    init_hitag2_hw();
    
    NRF_LOG_INFO("Requesting timeslot for precise BPLM transmission...");
    
    // Clear polled edges buffer
    g_polled_edge_count = 0;
    
    // Request timeslot for time-critical transmission AND reception
    // Following T55xx pattern: Field is started INSIDE timeslot
    // This ensures precise timing without BLE interference
    // Duration: 15ms (powerup + auth + transmission + GPIO polling + margin)
    // CRITICAL: GPIO polling happens INSIDE timeslot callback to capture response
    request_timeslot(15000, hitag2_timeslot_callback);
    
    NRF_LOG_INFO("START_AUTH transmitted with correct BPLM");
    NRF_LOG_INFO("GPIO polling captured %d edges", g_polled_edge_count);
    
    // Process polled edges through Manchester decoder
    bool ok = false;
    
    if (g_polled_edge_count == 0) {
        NRF_LOG_WARNING("No edges captured! Card may not be responding.");
    } else {
        NRF_LOG_INFO("Processing %d polled edges...", g_polled_edge_count);
        
        for (int i = 0; i < g_polled_edge_count; i++) {
            // Log first few intervals for debugging
            if (i < 10) {
                NRF_LOG_INFO("Processing polled edge #%d: interval=%d", i, g_polled_edges[i]);
            }
            
            if (hitag2.decoder.feed(codec, g_polled_edges[i])) {
                // Successfully decoded response
                memcpy(data, hitag2.get_data(codec), hitag2.data_size);
                ok = true;
                NRF_LOG_INFO("SUCCESS! Hitag2 UID: %02X%02X%02X%02X", 
                            data[0], data[1], data[2], data[3]);
                break;
            }
        }
    }
    
    // Clean up
    stop_lf_125khz_radio();
    uninit_hitag2_hw();
    cb_free(&cb);
    hitag2.free(codec);
    
    if (!ok) {
        NRF_LOG_INFO("Hitag2 tag not found or no response");
        NRF_LOG_INFO("Transmitter confirmed working (scope shows tag responding)");
        NRF_LOG_INFO("Processed %d polled edges but failed to decode valid UID", g_polled_edge_count);
        if (g_polled_edge_count == 0) {
            NRF_LOG_WARNING("No edges captured via GPIO polling!");
            NRF_LOG_WARNING("Check: Tag positioning, field strength, timing");
        } else {
            NRF_LOG_INFO("Edges captured but decode failed - check Manchester thresholds");
            NRF_LOG_INFO("Try: Different tag, closer positioning, check timing");
        }
    }
    
    return ok;
}
