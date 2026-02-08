#include "hitag.h"

#include <stdlib.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf_pwm.h"
#include "parity.h"
#include "protocols.h"
#include "t55xx.h"
#include "tag_base_type.h"
#include "utils/manchester.h"

#define NRF_LOG_MODULE_NAME hitag
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
NRF_LOG_MODULE_REGISTER();

// Hitag data sizes
#define HITAG_UID_DATA_SIZE (HITAG_UID_SIZE)
#define HITAG_DATA_SIZE (HITAG2_MAX_BLOCKS * HITAG_BLOCK_SIZE)
#define HITAG_RAW_SIZE 64

// Hitag encoding (based on Proxmark3 implementation):
// - Downstream (reader->tag): BPLM (Bi-Phase Mark coding)
//   * Transition at start of every bit period
//   * Logic 1: additional transition in middle of bit period
//   * Logic 0: no additional transition in middle
// - Upstream (tag->reader): Manchester encoding
// For emulation, we'll use the most common rate (RF/50 for Hitag2)
#define HITAG_T55XX_BLOCK_COUNT 3

// PWM sequence timing constants for RF/50 encoding
#define HITAG_PWM_DUTY_CYCLE 25      // 50% duty cycle (25 out of 50)
#define HITAG_PWM_COUNTER_TOP 50     // RF/50 timing
#define HITAG_PWM_POLARITY_BIT (1 << 15)  // MSB for polarity

// Period detection thresholds for Manchester demodulation at RF/50
// RELAXED: Lower thresholds for weak tag responses
// User reports tag responding but receiver not detecting - need more tolerance
#define HITAG_T_LOW 0x30      // Was 0x40 - lowered for weak signals
#define HITAG_T_HIGH 0x70     // Was 0x60 - increased range  
#define HITAG_T_JITTER 0x20   // Was 0x10 - doubled tolerance

// PWM sequence storage for Hitag2
static nrf_pwm_values_wave_form_t m_hitag2_pwm_seq_vals[HITAG_RAW_SIZE] = {};

static nrf_pwm_sequence_t const m_hitag2_pwm_seq = {
    .values.p_wave_form = m_hitag2_pwm_seq_vals,
    .length = NRF_PWM_VALUES_LENGTH(m_hitag2_pwm_seq_vals),
    .repeats = 0,
    .end_delay = 0,
};

const protocol *hitag_protocols[] = {
    &hitag2,
    &hitag2_paxton,
};

size_t hitag_protocols_size = ARRAY_SIZE(hitag_protocols);

typedef struct {
    uint8_t data[HITAG_DATA_SIZE];
    uint64_t raw;
    uint8_t raw_length;
    uint8_t biphase_state;  // For decoding BPLM from reader
    manchester *modem;      // For encoding Manchester upstream to reader
} hitag_codec;

// Hitag2 tag response uses Manchester encoding for upstream
// This is a simplified implementation for basic emulation
uint64_t hitag2_raw_data(uint8_t *uid) {
    // For basic Hitag2 emulation, we encode the UID
    // Real Hitag2 has complex crypto challenge-response
    uint64_t raw = 0;
    
    // Encode UID into raw format for Manchester transmission
    for (int i = 0; i < HITAG_UID_SIZE; i++) {
        raw <<= 8;
        raw |= uid[i];
    }
    
    return raw;
}

// Decode BPLM-encoded bit from reader (downstream)
// BPLM (Bi-Phase Mark coding):
// - Always a transition at the start of bit period
// - Logic 1: additional transition in middle of bit period (2 transitions total)
// - Logic 0: no additional transition in middle (1 transition total)
bool hitag2_decode_biplm_bit(hitag_codec *d, uint8_t half_bit) {
    // This is a simplified BPLM decoder
    // Real implementation would track edge timing and count transitions
    // Production code needs proper edge detection per Proxmark3 implementation
    if (d->biphase_state == 0) {
        d->biphase_state = half_bit;
        return false;  // Need second half
    } else {
        // Check for mid-bit transition
        // If state changed, it's a '1' (extra transition)
        // If state same, it's a '0' (no extra transition)
        // Note: This is a stub - actual bit value not currently used
        d->biphase_state = 0;
        return true;  // Complete bit decoded
    }
}

// Period detection for Manchester demodulation at RF/50
uint8_t hitag2_period(uint8_t interval) {
    // Simplified period detection for Hitag2
    // T0 = 8us, RF/50 means 50 RF cycles per bit
    // RELAXED thresholds to capture weak tag responses
    
    NRF_LOG_DEBUG("Period detect: interval=%d (0x%02X)", interval, interval);
    
    if (interval >= (HITAG_T_LOW - HITAG_T_JITTER) && interval <= (HITAG_T_LOW + HITAG_T_JITTER)) {
        NRF_LOG_DEBUG("  -> T_LOW (period 0)");
        return 0;
    }
    if (interval >= (HITAG_T_HIGH - HITAG_T_JITTER) && interval <= (HITAG_T_HIGH + HITAG_T_JITTER)) {
        NRF_LOG_DEBUG("  -> T_HIGH (period 1)");
        return 1;
    }
    NRF_LOG_DEBUG("  -> INVALID (period 3)");
    return 3;  // Invalid period
}

hitag_codec *hitag2_alloc(void) {
    hitag_codec *codec = malloc(sizeof(hitag_codec));
    codec->modem = malloc(sizeof(manchester));
    codec->modem->rp = hitag2_period;
    return codec;
}

void hitag_free(hitag_codec *d) {
    if (d->modem) {
        free(d->modem);
        d->modem = NULL;
    }
    free(d);
}

uint8_t *hitag_get_data(hitag_codec *d) {
    return d->data;
}

void hitag2_decoder_start(hitag_codec *d, uint8_t format) {
    memset(d->data, 0, HITAG_DATA_SIZE);
    d->raw = 0;
    d->raw_length = 0;
    d->biphase_state = 0;
    manchester_reset(d->modem);
}

bool hitag2_decode_feed(hitag_codec *d, bool bit) {
    d->raw <<= 1;
    d->raw_length++;
    if (bit) {
        d->raw |= 0x01;
    }
    
    // Log bit accumulation progress
    if (d->raw_length % 4 == 0) {
        NRF_LOG_DEBUG("Progress: %d bits, last nibble=0x%X", 
                     d->raw_length, (uint8_t)(d->raw & 0x0F));
    }
    
    if (d->raw_length < 32) {  // Wait for at least UID bits
        return false;
    }
    
    // Extract UID from raw data
    // This is a simplified decoder - real Hitag2 has authentication
    for (int i = 0; i < HITAG_UID_SIZE; i++) {
        d->data[i] = (d->raw >> ((HITAG_UID_SIZE - 1 - i) * 8)) & 0xFF;
    }
    
    NRF_LOG_INFO("Decoded 32-bit UID from raw: 0x%08X", (uint32_t)d->raw);
    
    return d->raw_length >= 32;
}

bool hitag2_decoder_feed(hitag_codec *d, uint16_t interval) {
    // Decode Manchester-encoded tag response (upstream)
    // Tag responds in Manchester encoding after START_AUTH
    
    NRF_LOG_DEBUG("Decoder feed: interval=%d (0x%04X)", interval, interval);
    
    bool bits[2] = {0};
    int8_t bitlen = 0;
    
    // Feed interval to Manchester decoder
    manchester_feed(d->modem, (uint8_t)interval, bits, &bitlen);
    
    if (bitlen == -1) {
        NRF_LOG_DEBUG("  -> Manchester decode failed, resetting");
        d->raw = 0;
        d->raw_length = 0;
        d->biphase_state = 0;
        return false;
    }
    
    NRF_LOG_DEBUG("  -> Decoded %d bit(s): %d %d", bitlen, bits[0], bitlen > 1 ? bits[1] : -1);
    
    for (int i = 0; i < bitlen; i++) {
        if (hitag2_decode_feed(d, bits[i])) {
            NRF_LOG_INFO("COMPLETE! Got %d bits total", d->raw_length);
            return true;
        }
    }
    return false;
}

const nrf_pwm_sequence_t *hitag2_modulator(hitag_codec *d, uint8_t *buf) {
    // Generate PWM sequence for Hitag2 tag response (upstream)
    // Tag uses Manchester encoding for upstream communication
    uint64_t data = hitag2_raw_data(buf);
    
    // Generate Manchester-encoded PWM sequence for tag response
    // Manchester encoding (IEEE 802.3 style used by Hitag):
    // Logic 0: high-to-low transition in middle of bit period
    // Logic 1: low-to-high transition in middle of bit period
    // Using RF/50 timing (50 RF cycles per bit period)
    
    for (int i = 0; i < 32; i++) {  // 32 bits for UID
        bool bit = IS_SET(data, 31 - i);
        
        if (bit) {
            // Logic 1: low-high (start low, transition to high)
            m_hitag2_pwm_seq_vals[i * 2].channel_0 = 0 | HITAG_PWM_DUTY_CYCLE;
            m_hitag2_pwm_seq_vals[i * 2].counter_top = HITAG_PWM_COUNTER_TOP;
            m_hitag2_pwm_seq_vals[i * 2 + 1].channel_0 = HITAG_PWM_POLARITY_BIT | HITAG_PWM_DUTY_CYCLE;
            m_hitag2_pwm_seq_vals[i * 2 + 1].counter_top = HITAG_PWM_COUNTER_TOP;
        } else {
            // Logic 0: high-low (start high, transition to low)
            m_hitag2_pwm_seq_vals[i * 2].channel_0 = HITAG_PWM_POLARITY_BIT | HITAG_PWM_DUTY_CYCLE;
            m_hitag2_pwm_seq_vals[i * 2].counter_top = HITAG_PWM_COUNTER_TOP;
            m_hitag2_pwm_seq_vals[i * 2 + 1].channel_0 = 0 | HITAG_PWM_DUTY_CYCLE;
            m_hitag2_pwm_seq_vals[i * 2 + 1].counter_top = HITAG_PWM_COUNTER_TOP;
        }
    }
    
    return &m_hitag2_pwm_seq;
}

// Hitag2 standard protocol
const protocol hitag2 = {
    .tag_type = TAG_TYPE_HITAG2,
    .data_size = HITAG_UID_DATA_SIZE,
    .alloc = (codec_alloc)hitag2_alloc,
    .free = (codec_free)hitag_free,
    .get_data = (codec_get_data)hitag_get_data,
    .modulator = (modulator)hitag2_modulator,
    .decoder = {
        .start = (decoder_start)hitag2_decoder_start,
        .feed = (decoder_feed)hitag2_decoder_feed,
    },
};

// Paxton Hitag2 variant (similar to standard but used in access control)
const protocol hitag2_paxton = {
    .tag_type = TAG_TYPE_HITAG2_PAXTON,
    .data_size = HITAG_UID_DATA_SIZE,
    .alloc = (codec_alloc)hitag2_alloc,
    .free = (codec_free)hitag_free,
    .get_data = (codec_get_data)hitag_get_data,
    .modulator = (modulator)hitag2_modulator,
    .decoder = {
        .start = (decoder_start)hitag2_decoder_start,
        .feed = (decoder_feed)hitag2_decoder_feed,
    },
};

// Encode Hitag2 UID to T55xx blocks
uint8_t hitag2_t55xx_writer(uint8_t *uid, uint32_t *blks) {
    // T55xx configuration for Hitag2
    // Upstream (tag->reader): Manchester encoding at RF/50
    // Downstream (reader->tag): BPLM (Bi-Phase Mark) handled by T55xx decoder
    blks[0] = T5577_HITAG2_CONFIG;
    
    // UID data in blocks 1 and 2
    blks[1] = (uid[0] << 24) | (uid[1] << 16) | (uid[2] << 8) | uid[3];
    blks[2] = 0x00000000;  // Reserved/config block
    
    return HITAG_T55XX_BLOCK_COUNT;
}
