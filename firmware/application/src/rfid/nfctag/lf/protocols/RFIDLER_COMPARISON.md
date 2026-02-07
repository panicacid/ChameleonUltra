# RFIDler vs ChameleonUltra Hitag2 Implementation Comparison

## Credits & Attribution

This analysis references two excellent open-source RFID projects:

### Proxmark3 Project
- **URL:** https://github.com/RfidResearchGroup/proxmark3
- **Files Analyzed:** 
  - `armsrc/hitag2.c`
  - `armsrc/hitag2.h`
  - `common/hitag2/hitag2_crypto.c`
- **License:** GPL-2.0
- **Used For:** Protocol timing reference, BPLM encoding details, START_AUTH implementation
- **Thanks:** Proxmark3 contributors for their detailed, well-documented implementation

### RFIDler Project
- **Author:** Adam Laurie and contributors
- **URL:** https://github.com/AdamLaurie/RFIDler
- **Files Analyzed:**
  - `firmware/Pic32/RFIDler.X/src/hitag.c`
  - `firmware/Pic32/RFIDler.X/src/hitag.h`
- **License:** BSD
- **Used For:** Alternative implementation analysis, timing tolerance insights, simpler approach comparison
- **Thanks:** Adam Laurie for RFIDler's elegant and educational implementation

**Note:** This document contains analysis and comparison only. No code was copied. ChameleonUltra implementation is original work based on protocol understanding.

---

## Executive Summary

After analyzing RFIDler's Hitag2 implementation and comparing it to our ChameleonUltra approach, **critical differences** were identified:

1. **RFIDler uses 10x LONGER timing** (1.6-2.3ms vs our 160-240μs)
2. **Built-in timing tolerance** (±160μs vs our microsecond precision)
3. **Simpler hardware approach** (Direct GPIO vs complex PWM)
4. **Proven to work** on real hardware

Our implementation may be over-engineered and fighting hardware limitations unnecessarily.

---

## Hardware Comparison

| Aspect | RFIDler (PIC32) | ChameleonUltra (NRF52) |
|--------|-----------------|------------------------|
| Microcontroller | PIC32MX695F512H | NRF52840 |
| LF Generation | Direct GPIO coil control | PWM Module |
| Control Method | `COIL_ON()` / `COIL_OFF()` macros | `start_lf_125khz_radio()` / `stop_lf_125khz_radio()` |
| Complexity | ~50 lines of code | ~200 lines of code |
| Settling Time | Not needed (direct GPIO) | 15μs PWM ramp up/down |

---

## RFIDler Implementation Analysis

### Code Structure (from RFIDler/firmware/Pic32/RFIDler.X/src/hitag.c)

```c
// Direct coil control via GPIO
#define COIL_ON()   mLED_1_On()
#define COIL_OFF()  mLED_1_Off()

// Timing constants (in 125kHz carrier periods)
#define RWD_TIME_0      208  // Bit 0 duration
#define RWD_TIME_1      288  // Bit 1 duration
#define RWD_TIME_FUZZ   20   // ±20 periods tolerance!

// BPLM bit transmission
void hitag2_send_bit(BOOL one) {
    COIL_OFF();                              // Start with field OFF
    DelayClock(RWD_TIME_0);                  // Wait fixed time
    if(one)
        DelayClock(RWD_TIME_1 - RWD_TIME_0); // Extra delay for '1'
    COIL_ON();                               // Field back ON
}
```

### Key Observations

**1. Timing is MUCH Longer**
- Bit 0: 208 periods × 8μs = **1.664ms** (vs our 160μs)
- Bit 1: 288 periods × 8μs = **2.304ms** (vs our 240μs)
- **10x longer than what we're using!**

**2. Built-in Tolerance**
- `RWD_TIME_FUZZ = 20` periods = ±160μs
- This is larger than our ENTIRE bit duration!
- Shows timing doesn't need microsecond precision

**3. Pattern Difference**
- RFIDler: Fixed OFF time (208 periods), variable total duration
- Us: Fixed total duration (160/240μs), variable ON time
- Both should work per protocol, but RFIDler's is proven

**4. Simplicity**
- Direct GPIO control
- No PWM complexity
- No settling delays needed
- Just turn coil on/off

---

## ChameleonUltra Implementation

### Current Approach

```c
// Complex PWM control
#define HITAG2_PWM_SETTLE_US     15   // Settling delay
#define HITAG2_BPLM_PULSE_US     48   // OFF pulse
#define HITAG2_BPLM_BIT0_HIGH_US 112  // ON time bit 0
#define HITAG2_BPLM_BIT1_HIGH_US 192  // ON time bit 1

static void hitag2_send_bit(uint8_t bit) {
    stop_lf_125khz_radio();           // Stop PWM
    bsp_delay_us(15);                 // Let PWM ramp down
    bsp_delay_us(33);                 // Clean OFF time
    start_lf_125khz_radio();          // Start PWM
    bsp_delay_us(15);                 // Let PWM stabilize
    bsp_delay_us(bit ? 177 : 97);     // Clean ON time
}
```

### Issues Identified

**1. Fighting PWM Limitations**
- PWM not designed for rapid on/off switching
- Need settling delays for hardware to catch up
- Complex timing calculations

**2. Microsecond Precision**
- Trying for exact 160/240μs timing
- But RFIDler shows ±160μs tolerance is acceptable!
- Over-engineered for what protocol actually needs

**3. Very Short Durations**
- 48μs pulse may be too short for PWM
- Hardware latency eats into actual OFF time
- Field may never fully stop/start

---

## T55xx Assumption Analysis

### The Assumption

We've been using T55xx writer as reference, assuming:
- It works correctly on ChameleonUltra
- Its patterns are good to copy
- If T55xx works, our pattern should work

### Reality Check

**T55xx Timing (from lf_t55xx_data.c):**
```c
#define T55XX_WRITE_SEQUENCE_1      432  // 432μs
#define T55XX_WRITE_SEQUENCE_0      192  // 192μs
#define T55XX_WRITE_SEQUENCE_GAP    72   // 72μs gaps

void t55xx_send_gap(uint32_t nus) {
    stop_lf_125khz_radio();
    bsp_delay_us(nus);
    start_lf_125khz_radio();
}
```

**Key Differences:**
- T55xx uses 72μs gaps (vs our 48μs)
- T55xx uses 192-432μs ON times (vs our 112-192μs)
- **T55xx is 3-10x SLOWER than our Hitag2 timing**

**Why This Matters:**
- T55xx may work DESPITE PWM issues, not BECAUSE pattern is good
- Longer timing = less sensitive to PWM latency
- 72μs gap gives PWM time to actually stop
- 192μs+ ON gives PWM time to actually start and stabilize

**Conclusion:** T55xx is NOT a good reference for fast protocols!

### Questions We Should Answer

1. **Does T55xx actually work on ChameleonUltra?**
   - Has anyone verified successful writes?
   - Any test data or user reports?

2. **If T55xx works, is it because:**
   - Pattern is correct? OR
   - Timing is slow enough to hide PWM issues?

3. **Should we copy T55xx pattern for Hitag2?**
   - NO - timing requirements are very different
   - T55xx is much slower protocol
   - Need different approach

---

## Critical Problem: PWM Module Limitations

### What PWM is Designed For

- Continuous operation (LED dimming, motor control)
- Stable, constant duty cycles
- Gradual changes
- Long-term operation

### What We're Asking PWM to Do

- Rapid on/off switching at microsecond scale
- Field modulation for communication
- Time-critical protocol timing
- Precise start/stop synchronization

**These are fundamentally incompatible!**

### NRF52 PWM Characteristics

From NRF52840 Product Specification Section 6.32:

```c
// PWM configuration (lf_125khz_radio.c)
config.base_clock = NRF_PWM_CLK_500kHz;
config.top_value = 4;
// Output: 500kHz / 4 = 125kHz

// Start with looping
nrfx_pwm_simple_playback(&m_pwm, &seq, 1, NRFX_PWM_FLAG_LOOP);

// Stop with wait
nrfx_pwm_stop(&m_pwm, true);
```

**Problems:**
1. **Looping mode** means continuous operation
2. **Stop command** must wait for current sequence to finish
3. **Start command** must load and begin new sequence
4. **Overhead:** Unknown latency, likely 10-50μs per operation
5. **No specification** for start/stop timing in datasheet

### Real PWM Behavior (Likely)

```
stop_lf_125khz_radio() called
  ↓ (8-16μs) - Wait for sequence to finish
Actually stops
  ↓ (48μs) - Our delay
start_lf_125khz_radio() called
  ↓ (8-16μs) - Load and start new sequence
Actually running

Total: ~70-80μs instead of our intended 48μs
```

**Result:** Timing is off, PM3 can't detect proper BPLM pattern

---

## Why RFIDler's Approach Works Better

### 1. Direct Control

```c
COIL_OFF();  // GPIO = 0 immediately
// ... delay ...
COIL_ON();   // GPIO = 1 immediately
```

- No PWM overhead
- No sequence loading
- No waiting for hardware
- Instant response

### 2. Longer Timing

- 1.6-2.3ms durations
- Easy to detect
- Less sensitive to small errors
- More margin for hardware

### 3. Built-in Tolerance

- `RWD_TIME_FUZZ = ±160μs`
- Protocol accepts loose timing
- No need for microsecond precision
- Robust in real world

### 4. Simplicity

- Fewer lines of code
- Fewer things to go wrong
- Easier to debug
- Proven working

---

## Recommended Approaches

### Option A: Increase Timing (Quick Test)

Match RFIDler's proven timing:

```c
// Test with 5-10x longer durations
#define HITAG2_BPLM_PULSE_US     400   // was 48, now 400 (5x)
#define HITAG2_BPLM_BIT0_HIGH_US 800   // was 112, now 800 (7x)
#define HITAG2_BPLM_BIT1_HIGH_US 1600  // was 192, now 1600 (8x)
```

**Advantages:**
- Quick to test
- More margin for PWM latency
- More detectable by PM3
- Proven timing from RFIDler

**Disadvantages:**
- Slower transmission
- Not exactly per spec (but within tolerance)

### Option B: Simplified Implementation (Current Alternative)

Already added to lf_hitag2_data.c:

```c
static void hitag2_send_bit_simple(uint8_t bit) {
    // No settling delays, just longer times
    stop_lf_125khz_radio();
    bsp_delay_us(100);  // Longer pulse
    start_lf_125khz_radio();
    bsp_delay_us(bit ? 300 : 200);  // Longer ON
}
```

**Advantages:**
- Simpler to debug
- Easier to measure with scope
- More PWM settling time
- Still faster than RFIDler

### Option C: Direct GPIO Control (Major Rewrite)

Generate 125kHz manually:

```c
void generate_125khz_pulse(uint32_t duration_us) {
    uint32_t cycles = duration_us * 125 / 1000;  // Number of 125kHz cycles
    
    for (uint32_t i = 0; i < cycles; i++) {
        GPIO_SET(LF_ANT_DRIVER);
        delay_cycles(32);  // ~4μs ON (64MHz / 32 / 2)
        GPIO_CLR(LF_ANT_DRIVER);
        delay_cycles(32);  // ~4μs OFF
    }
}
```

**Advantages:**
- Direct control like RFIDler
- No PWM latency
- Predictable timing

**Disadvantages:**
- CPU intensive
- Requires tight timing loop
- May not generate clean 125kHz
- Significant code changes

### Option D: Hybrid Approach

Use PWM for continuous carrier, GPIO for modulation:

```c
// Configure PWM to drive GPIO, then override for modulation
// Complex but combines benefits of both
```

---

## PicoScope Testing Protocol

User has PicoScope available - use it to measure reality!

### Test 1: PWM Baseline

```c
// Measure clean 125kHz carrier
start_lf_125khz_radio();
bsp_delay_us(10000);  // 10ms
stop_lf_125khz_radio();
```

**Measure:**
- Is 125kHz actually 125kHz?
- Is duty cycle 50%?
- How clean is the waveform?

### Test 2: PWM Start/Stop Latency

```c
// Measure actual latency
for (int i = 0; i < 10; i++) {
    start_lf_125khz_radio();
    bsp_delay_us(100);
    stop_lf_125khz_radio();
    bsp_delay_us(100);
}
```

**Measure:**
- Time from `start` call to actual carrier
- Time from `stop` call to actual silence
- Is 100μs pulse actually 100μs OFF?

### Test 3: Current BPLM Implementation

```c
// Measure our actual transmission
hitag2_send_start_auth();  // 5 bits
```

**Measure:**
- Actual pulse duration (expect 48μs)
- Actual ON duration (expect 112/192μs)
- Are settling delays visible?
- Total timing vs expected

### Test 4: Simplified Implementation

```c
// Measure simpler approach
hitag2_send_start_auth_simple();  // 5 bits, longer timing
```

**Measure:**
- Compare to current implementation
- Is it cleaner?
- Better defined transitions?

### Test 5: T55xx Verification

```c
// Measure T55xx for comparison
bsp_delay_us(432);  // T55xx ON
t55xx_send_gap(72); // T55xx gap
```

**Measure:**
- Actual gap duration
- Compare PWM behavior to Hitag2
- Verify T55xx actually works

### Expected Results

**If PWM latency is the problem:**
- Measured times will be shorter than code specifies
- Start/stop commands will show visible delay
- Solution: Use longer times or different approach

**If timing is correct:**
- Measurements match code exactly
- Problem must be elsewhere (signal strength, PM3 decoder, etc.)

**If PWM doesn't work:**
- No modulation visible
- Hardware or execution problem

---

## Conclusions

### What RFIDler Teaches Us

1. **Simpler is better** - Direct GPIO vs complex PWM
2. **Timing can be looser** - ±160μs tolerance acceptable
3. **Longer is more detectable** - 1.6-2.3ms vs 0.16-0.24ms
4. **Proven approach works** - Don't reinvent unnecessarily

### What We Learned About T55xx

1. **Not a good reference** - 10x slower timing
2. **May work despite issues** - Slow enough to hide PWM problems
3. **Different protocol needs** - Can't directly copy pattern

### What We Need to Do

1. **Measure with PicoScope** - Get real data, stop guessing
2. **Try longer timing** - Match RFIDler's proven approach
3. **Consider alternatives** - GPIO control if PWM inadequate
4. **Stop assuming** - Verify everything with measurements

### Next Steps

1. User tests with PicoScope (5 test scenarios above)
2. Based on measurements:
   - If PWM latency confirmed: Use longer timing
   - If timing correct: Investigate other factors
   - If PWM broken: Switch to GPIO or hybrid
3. Implement solution based on data
4. Re-test with PM3 sniffer
5. Verify with real tags

---

## References

### Source Code Analyzed

**Proxmark3:**
- https://github.com/RfidResearchGroup/proxmark3/blob/master/armsrc/hitag2.c
- https://github.com/RfidResearchGroup/proxmark3/blob/master/armsrc/hitag2.h

**RFIDler:**
- https://github.com/AdamLaurie/RFIDler/blob/master/firmware/Pic32/RFIDler.X/src/hitag.c
- https://github.com/AdamLaurie/RFIDler/blob/master/firmware/Pic32/RFIDler.X/src/hitag.h

### Documentation

- NRF52840 Product Specification v1.1
- Hitag2 Protocol Documentation: http://www.proxmark.org/files/Documents/125%20kHz%20-%20Hitag/
- NRFX PWM Driver Documentation

### Special Thanks

- **Proxmark3 contributors** for comprehensive, well-documented implementation
- **Adam Laurie** for RFIDler's elegant and educational approach
- **Open source community** for knowledge sharing

Without these references, this implementation would have been much more difficult!

---

*This document is part of the ChameleonUltra Hitag2 implementation. Analysis performed February 2026.*
