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
 *   - Result: Only 18μs clean gap detected (need 48μs)
 * 
 * Solution: Extend pulse time to compensate for ring-down
 *   - Total pulse: 69μs = 21μs (ring-down) + 48μs (clean gap)
 *   - PM3 now sees required 48μs clean gap ✓
 * 
 * PWM Overhead Compensation (PicoScope Analysis #2)
 * ==================================================
 * Measured actual bit timing (falling edge to falling edge):
 *   - Bit 1 measured: 250.4μs (should be 240μs)
 *   - Overhead: ~10μs from PWM start/stop execution time
 * 
 * Solution: Reduce ON times by 10μs to compensate
 *   - Keep pulse at 69μs (critical for ring-down)
 *   - Reduce HIGH times to account for PWM overhead
 * 
 * Final compensated timing:
 *   Bit 0: OFF(69μs) + ON(81μs) = 150μs theory → ~160μs actual ✓
 *   Bit 1: OFF(69μs) + ON(161μs) = 230μs theory → ~240μs actual ✓
 */

// Fixed pulse (OFF) duration - COMPENSATED FOR RING-DOWN
#define HITAG2_BPLM_PULSE_US     69   // 48μs spec + 21μs ring-down compensation

// Total bit durations per Hitag2 specification
#define HITAG2_BPLM_0_TIME       160  // 20 Tc = 160μs (bit 0 total)
#define HITAG2_BPLM_1_TIME       240  // 30 Tc = 240μs (bit 1 total)

// Calculated ON times: Compensated for both ring-down AND PWM overhead
// Theory: TOTAL - PULSE - OVERHEAD, but we subtract from ON time to get actual = spec
#define HITAG2_BPLM_BIT0_HIGH_US 81   // 69 + 81 = 150μs theory → ~160μs actual (10μs overhead)
#define HITAG2_BPLM_BIT1_HIGH_US 161  // 69 + 161 = 230μs theory → ~240μs actual (10μs overhead)

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
