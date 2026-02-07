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
#define HITAG_T_0_US                160  // 20 Tc = 160μs (bit 0)
#define HITAG_T_1_US                240  // 30 Tc = 240μs (bit 1)
#define HITAG_T_GAP_US               32  // 4 Tc = 32μs (gap between bits)

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
 * Control field on/off for BPLM modulation
 */
static inline void hitag2_field_on(void) {
    start_lf_125khz_radio();
}

static inline void hitag2_field_off(void) {
    stop_lf_125khz_radio();
}

/**
 * Send a single bit using true BPLM (Bi-Phase Mark) encoding
 * 
 * BPLM encoding (from Hitag2 specification):
 * - Always a transition (field change) at the start of each bit period
 * - Bit 0: Only the start transition (one transition total)
 * - Bit 1: Start transition + mid-bit transition (two transitions total)
 * 
 * Implementation:
 * - Track field state to ensure proper transitions
 * - Use actual field ON/OFF for transitions (not just timing)
 * - This creates detectable signal that Proxmark3 can sniff
 * 
 * @param bit The bit value (0 or 1)
 * @param field_state Pointer to current field state (true=ON, false=OFF)
 */
static void hitag2_send_bit(uint8_t bit, bool *field_state) {
    // BPLM: Always transition at start of bit
    *field_state = !(*field_state);
    if (*field_state) {
        hitag2_field_on();
    } else {
        hitag2_field_off();
    }
    
    if (bit & 0x01) {
        // Bit 1: Two transitions
        // First half of bit period
        bsp_delay_us(HITAG_T_1_US / 2);
        
        // Mid-bit transition
        *field_state = !(*field_state);
        if (*field_state) {
            hitag2_field_on();
        } else {
            hitag2_field_off();
        }
        
        // Second half of bit period
        bsp_delay_us(HITAG_T_1_US / 2);
    } else {
        // Bit 0: Only start transition, no mid-bit transition
        bsp_delay_us(HITAG_T_0_US);
    }
}

/**
 * Send START_AUTH command (5 bits: 11000)
 * This initiates communication with tag in public mode
 * 
 * @param field_state Pointer to current field state for BPLM tracking
 */
static void hitag2_send_start_auth(bool *field_state) {
    uint8_t cmd = HITAG2_START_AUTH_CMD;
    
    NRF_LOG_INFO("Transmitting START_AUTH with BPLM modulation...");
    
    // Send 5 bits MSB first: 1, 1, 0, 0, 0
    // Bit positions in 0xC0 (11000000):
    // Bit 7: 1, Bit 6: 1, Bit 5: 0, Bit 4: 0, Bit 3: 0
    for (int i = 7; i >= 3; i--) {
        uint8_t bit = (cmd >> i) & 0x01;
        hitag2_send_bit(bit, field_state);
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
    NRF_LOG_INFO("Hitag2 RTF protocol with BPLM modulation starting...");
    
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
    
    // Track field state for BPLM encoding
    // Start with field OFF, first transition will turn it ON
    bool field_state = false;
    
    // Start LF field - this powers the tag and lights LED
    // Field must stay on during entire transaction
    start_lf_125khz_radio();
    field_state = true;
    
    // Step 1: Wait for tag to power up (2.5ms)
    NRF_LOG_INFO("Waiting for tag powerup...");
    bsp_delay_us(HITAG_T_WAIT_POWERUP_US);
    
    // Step 2: Wait to be in START_AUTH window (464μs)
    bsp_delay_us(HITAG_T_WAIT_START_AUTH_US);
    
    // Step 3: Send START_AUTH command (5 bits: 11000) with BPLM modulation
    NRF_LOG_INFO("Sending START_AUTH with BPLM...");
    hitag2_send_start_auth(&field_state);
    
    // After transmission, ensure field is ON for tag response
    if (!field_state) {
        hitag2_field_on();
        field_state = true;
    }
    
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
        NRF_LOG_INFO("BPLM modulation transmitted - check with sniffer");
    }
    
    return ok;
}
