#include "lf_hitag2_data.h"

#include "bsp_delay.h"
#include "bsp_time.h"
#include "circular_buffer.h"
#include "lf_125khz_radio.h"
#include "lf_reader_data.h"
#include "protocols/hitag.h"

#define NRF_LOG_MODULE_NAME hitag2_reader
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
NRF_LOG_MODULE_REGISTER();

// Hitag2 timing constants (in microseconds)
// Based on Proxmark3 hitag2.c and converted to ChameleonUltra timing
// At 125kHz: 1 carrier period (Tc) = 8μs
#define HITAG_T_WAIT_POWERUP_US    2504  // 313 Tc = 2.504ms
#define HITAG_T_WAIT_START_AUTH_US  464  // 58 Tc = 464μs  
#define HITAG_T_WAIT_RESPONSE_US   1600  // 200 Tc = 1.6ms

// BPLM encoding using gap modulation (like T55xx)
// Instead of toggling field ON/OFF rapidly (too fast for PWM),
// we keep field ON and create gaps for transitions
#define HITAG_T_0_HIGH_US          140  // 18 Tc = 144μs field ON for bit 0
#define HITAG_T_0_GAP_US            20  // 2-3 Tc = 16-24μs gap for bit 0  
#define HITAG_T_1_HIGH_US          100  // 12-13 Tc = ~100μs field ON for first half of bit 1
#define HITAG_T_1_GAP_US            20  // 2-3 Tc = 16-24μs gap for bit 1 transitions
#define HITAG_T_1_HIGH2_US         100  // 12-13 Tc = ~100μs field ON for second half of bit 1

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
 * Send a gap (field OFF briefly) like T55xx
 * This creates a detectable transition in the field
 */
static void hitag2_send_gap(uint32_t gap_us) {
    stop_lf_125khz_radio();
    bsp_delay_us(gap_us);
    start_lf_125khz_radio();
}

/**
 * Send a single bit using BPLM encoding via gap modulation
 * 
 * BPLM encoding adapted for ChameleonUltra hardware:
 * - Field stays ON most of the time (powers tag)
 * - Transitions created by brief gaps (field OFF)
 * - Bit 0: Field ON, one gap, field ON (total ~160μs)
 * - Bit 1: Field ON, gap, field ON, gap, field ON (total ~240μs)
 * 
 * This approach works like T55xx writing (proven working).
 * Much more reliable than rapid PWM start/stop toggling.
 * 
 * @param bit The bit value (0 or 1)
 */
static void hitag2_send_bit(uint8_t bit) {
    if (bit & 0x01) {
        // Bit 1: Two transitions (two gaps)
        // Field ON for first half
        bsp_delay_us(HITAG_T_1_HIGH_US);
        // First gap (transition 1)
        hitag2_send_gap(HITAG_T_1_GAP_US);
        // Field ON for second half
        bsp_delay_us(HITAG_T_1_HIGH2_US);
        // Second gap (transition 2)
        hitag2_send_gap(HITAG_T_1_GAP_US);
    } else {
        // Bit 0: One transition (one gap)
        // Field ON for bit duration
        bsp_delay_us(HITAG_T_0_HIGH_US);
        // Single gap (transition)
        hitag2_send_gap(HITAG_T_0_GAP_US);
    }
}

/**
 * Send START_AUTH command (5 bits: 11000)
 * This initiates communication with tag in public mode
 */
static void hitag2_send_start_auth(void) {
    uint8_t cmd = HITAG2_START_AUTH_CMD;
    
    NRF_LOG_INFO("Transmitting START_AUTH with gap modulation...");
    
    // Send 5 bits MSB first: 1, 1, 0, 0, 0
    // Bit positions in 0xC0 (11000000):
    // Bit 7: 1, Bit 6: 1, Bit 5: 0, Bit 4: 0, Bit 3: 0
    for (int i = 7; i >= 3; i--) {
        uint8_t bit = (cmd >> i) & 0x01;
        hitag2_send_bit(bit);
    }
    
    NRF_LOG_INFO("START_AUTH transmission complete");
}

/**
 * Attempt to read Hitag2 tag UID using RTF protocol
 * 
 * Protocol flow adapted for ChameleonUltra hardware:
 * 1. Start field to power tag (like all LF protocols)
 * 2. Wait for tag powerup
 * 3. Send START_AUTH command via field modulation (like T55xx)
 * 4. Receive Manchester response via GPIO interrupts (like EM410x)
 * 
 * Based on:
 * - Proxmark3 hitag2.c for protocol timing
 * - ChameleonUltra T55xx for transmission method
 * - ChameleonUltra EM410x for reception method
 */
bool hitag2_read(uint8_t *data, uint32_t timeout_ms) {
    NRF_LOG_INFO("Hitag2 RTF protocol with gap modulation starting...");
    
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
    
    // Start LF field - this powers the tag and lights LED
    start_lf_125khz_radio();
    
    // Step 1: Wait for tag to power up (2.5ms)
    NRF_LOG_INFO("Waiting for tag powerup...");
    bsp_delay_us(HITAG_T_WAIT_POWERUP_US);
    
    // Step 2: Wait to be in START_AUTH window (464μs)
    bsp_delay_us(HITAG_T_WAIT_START_AUTH_US);
    
    // Step 3: Send START_AUTH command (5 bits: 11000) with gap modulation
    // Field is already ON, gaps will create transitions
    NRF_LOG_INFO("Sending START_AUTH with gap modulation...");
    hitag2_send_start_auth();
    
    // Field should be ON after transmission (last operation was send_gap which ends with field ON)
    // No need to check field state
    
    // Step 4: Wait for tag response
    bsp_delay_us(HITAG_T_WAIT_RESPONSE_US);
    
    // Step 5: Try to decode response (32-bit UID, Manchester encoded)
    bool ok = false;
    autotimer *p_at = bsp_obtain_timer(0);
    
    // Process received edge timings
    while (!ok && NO_TIMEOUT_1MS(p_at, timeout_ms)) {
        uint16_t val = 0;
        while (!ok && NO_TIMEOUT_1MS(p_at, timeout_ms) && cb_pop_front(&cb, &val)) {
            if (hitag2.decoder.feed(codec, val)) {
                // Successfully decoded response
                memcpy(data, hitag2.get_data(codec), hitag2.data_size);
                ok = true;
                NRF_LOG_INFO("Hitag2 UID: %02X%02X%02X%02X", 
                            data[0], data[1], data[2], data[3]);
                break;
            }
        }
    }
    
    bsp_return_timer(p_at);
    
    // Clean up
    stop_lf_125khz_radio();
    uninit_hitag2_hw();
    cb_free(&cb);
    hitag2.free(codec);
    
    if (!ok) {
        NRF_LOG_INFO("Hitag2 tag not found or no response");
        NRF_LOG_INFO("Gap modulation transmitted - check with sniffer");
    }
    
    return ok;
}
