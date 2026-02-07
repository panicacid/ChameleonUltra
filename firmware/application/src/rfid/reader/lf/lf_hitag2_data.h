#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Hitag2 BPLM Timing Constants
 * 
 * BPLM (Bi-Phase Mark) encoding timing per Hitag2 specification:
 * - Bit 0: Total duration 160μs (20 Tc @ 125kHz)
 * - Bit 1: Total duration 240μs (30 Tc @ 125kHz)
 * 
 * Implementation uses gap modulation:
 * - Field stays ON for most of bit duration
 * - Brief gaps (field OFF) create transitions
 * - HIGH_TIME + GAP_TIME = TOTAL_TIME
 * 
 * Example for Bit 0:
 *   HITAG2_BPLM_BIT0_HIGH_US (140) + HITAG2_BPLM_LOW_TIME (20) = 160μs ✓
 */

// Gap (LOW) duration - when field is OFF
#define HITAG2_BPLM_LOW_TIME    20   // 20μs gap duration

// Bit 0 timing: HIGH + LOW = 160μs total
#define HITAG2_BPLM_0_TIME      160  // Total duration for bit 0
#define HITAG2_BPLM_BIT0_HIGH_US (HITAG2_BPLM_0_TIME - HITAG2_BPLM_LOW_TIME)  // 140μs

// Bit 1 timing: HIGH + LOW + HIGH + LOW = 240μs total
#define HITAG2_BPLM_1_TIME      240  // Total duration for bit 1
#define HITAG2_BPLM_BIT1_HIGH_US ((HITAG2_BPLM_1_TIME - 2*HITAG2_BPLM_LOW_TIME) / 2)  // 100μs per segment

/**
 * Read Hitag2 tag UID
 * 
 * @param data      Buffer to store the 4-byte UID
 * @param timeout_ms Timeout in milliseconds
 * @return true if tag was read successfully, false otherwise
 * 
 * Note: Hitag2 is a Reader-Talk-First (RTF) protocol that requires
 * active interrogation. This is different from Tag-Talk-First protocols
 * like EM410x where the tag continuously broadcasts.
 */
bool hitag2_read(uint8_t *data, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
