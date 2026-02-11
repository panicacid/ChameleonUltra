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
 * CRITICAL: Antenna Ring-Down Compensation (PicoScope Analysis)
 * ============================================================
 * When PWM stops, the LC antenna circuit rings down (damped oscillation).
 * PicoScope measurements on ChameleonUltra hardware:
 *   - Ring-down duration: 21μs
 *   - Ring-down amplitude: 34.95mV (18.3% of 190.9mV carrier)
 *   - PM3 zero-crossing detector sees ring-down as "carrier present"
 * 
 * PWM Overhead Compensation (PicoScope Analysis #2)
 * ==================================================
 * Measured actual bit timing (falling edge to falling edge):
 *   - Bit 1 measured: 250.4μs (should be 240μs per spec)
 *   - Total measured: 978μs (should be 928μs per HT2protocol.pdf)
 *   - Overhead: ~10μs from PWM start/stop execution time
 * 
 * HT2protocol.pdf Specification Calibration (Final)
 * ==================================================
 * Cross-referenced spec with measurements:
 *   - Bit 0 spec: 160μs (20 Tc) ✓
 *   - Bit 1 spec: 224μs (28 Tc) NOT 240μs!
 *   - START_AUTH total: 2×224 + 3×160 = 928μs
 * 
 * Final spec-calibrated timing:
 *   Pulse: 58μs (tightened from 69μs, covers ~21μs ring-down + 37μs clean)
 *   Bit 0: OFF(58μs) + ON(92μs) = 150μs theory → ~160μs actual ✓
 *   Bit 1: OFF(58μs) + ON(156μs) = 214μs theory → ~224μs actual ✓
 *   Total: 2×224 + 3×160 = 928μs (exact spec compliance) ✓
 */

// Fixed pulse (OFF) duration - SPEC-CALIBRATED
#define HITAG2_BPLM_PULSE_US     58   // Tightened to match spec (21μs ring-down + 37μs clean)

// Total bit durations per HT2protocol.pdf specification
#define HITAG2_BPLM_0_TIME       160  // 20 Tc = 160μs (bit 0 total)
#define HITAG2_BPLM_1_TIME       224  // 28 Tc = 224μs (bit 1 total) - SPEC NOT 240!

// Calculated ON times: Spec-calibrated with PWM overhead compensation
// Formula: TARGET - PULSE - OVERHEAD, subtract from ON time to compensate
#define HITAG2_BPLM_BIT0_HIGH_US 92   // 58 + 92 = 150μs theory → ~160μs actual (10μs overhead)
#define HITAG2_BPLM_BIT1_HIGH_US 156  // 58 + 156 = 214μs theory → ~224μs actual (10μs overhead)

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
