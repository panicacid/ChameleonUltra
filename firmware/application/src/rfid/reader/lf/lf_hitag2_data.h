#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Hitag2 BPLM (Binary Pulse Length Modulation) Timing Constants
 * 
 * Based on Proxmark3 hitag2.c and Hitag2 specification.
 * At 125kHz: 1 carrier period (Tc) = 8μs
 * 
 * BPLM encoding: Each bit starts with PULSE (field OFF), then field ON
 * - Bit value is encoded in the length of the ON period
 * - NOT gap modulation (that's T55xx)
 * - Pattern: |___PULSE___|------ON------|
 * 
 * From Proxmark3:
 *   #define HITAG_T_LOW  6   // T_LOW should be 4..10 carrier periods
 *   #define HITAG_T_0    20  // T[0] should be 18..22 carrier periods  
 *   #define HITAG_T_1    30  // T[1] should be 26..30 carrier periods
 * 
 * Timing breakdown:
 *   Bit 0: OFF(48μs/6Tc) + ON(112μs/14Tc) = 160μs/20Tc total
 *   Bit 1: OFF(48μs/6Tc) + ON(192μs/24Tc) = 240μs/30Tc total
 */

// Fixed pulse (OFF) duration for all bits
#define HITAG2_BPLM_PULSE_US     48   // 6 Tc = 48μs (field OFF)

// Total bit durations per Hitag2 specification
#define HITAG2_BPLM_0_TIME       160  // 20 Tc = 160μs (bit 0 total)
#define HITAG2_BPLM_1_TIME       240  // 30 Tc = 240μs (bit 1 total)

// Calculated ON times: TOTAL - PULSE = ON_TIME
#define HITAG2_BPLM_BIT0_HIGH_US (HITAG2_BPLM_0_TIME - HITAG2_BPLM_PULSE_US)  // 112μs (14 Tc)
#define HITAG2_BPLM_BIT1_HIGH_US (HITAG2_BPLM_1_TIME - HITAG2_BPLM_PULSE_US)  // 192μs (24 Tc)

// PWM hardware settling time
// NRF52 PWM needs time to ramp up/down cleanly (~1-2 PWM cycles at 125kHz = 8-16μs)
// We use 15μs as empirically determined value for clean transitions
#define HITAG2_PWM_SETTLE_US     15   // Time for PWM to stabilize after start/stop

/**
 * Read Hitag2 tag UID using RTF protocol with BPLM encoding
 * 
 * @param data      Buffer to store the 4-byte UID
 * @param timeout_ms Timeout in milliseconds
 * @return true if tag was read successfully, false otherwise
 * 
 * Note: Hitag2 is a Reader-Talk-First (RTF) protocol using BPLM encoding.
 * This requires active interrogation with precise timing, implemented using
 * the timeslot API to prevent BLE interference.
 */
bool hitag2_read(uint8_t *data, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
