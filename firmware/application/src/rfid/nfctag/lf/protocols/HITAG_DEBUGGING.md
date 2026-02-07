# Hitag2 Implementation Debugging Guide

## Issue: Proxmark3 Detecting Zero Auth Attempts

This document provides comprehensive debugging guidance for the Hitag2 implementation when Proxmark3 sniffer reports 0 auth attempts.

## Hardware Verification ✓

### NRF52840 Configuration

**PWM Configuration (lf_125khz_radio.c):**
```c
Base Clock: 500kHz
Top Value: 4
Output Frequency: 500kHz / 4 = 125kHz ✓
Duty Cycle: 2/4 = 50% ✓
Pin: LF_ANT_DRIVER (inverted)
```

**PWM Control:**
```c
start_lf_125khz_radio():
  - nrfx_pwm_simple_playback(&m_pwm, &seq, 1, NRFX_PWM_FLAG_LOOP)
  - Starts continuous 125kHz carrier

stop_lf_125khz_radio():
  - nrfx_pwm_stop(&m_pwm, true)  // wait=true
  - Stops PWM, waits for completion
```

### Timing Constants

All timing verified against Proxmark3 and Hitag2 specification:

| Constant | Value | Carrier Periods | Purpose |
|----------|-------|-----------------|---------|
| HITAG_T_WAIT_POWERUP | 2504μs | 313 Tc | Tag powerup |
| HITAG_T_WAIT_START_AUTH | 464μs | 58 Tc | AUTH window |
| HITAG2_BPLM_PULSE | 48μs | 6 Tc | OFF pulse |
| HITAG2_BPLM_BIT0_HIGH | 112μs | 14 Tc | Bit 0 ON |
| HITAG2_BPLM_BIT1_HIGH | 192μs | 24 Tc | Bit 1 ON |
| HITAG2_PWM_SETTLE | 15μs | ~2 Tc | HW settling |

**Total Timing:**
- Bit 0: 48 + 112 = 160μs (20 Tc) ✓
- Bit 1: 48 + 192 = 240μs (30 Tc) ✓
- START_AUTH: 5 bits = 960μs ✓

## Diagnostic Steps

### 1. Verify LED Behavior

**Expected:**
- LED lights up when `lf hitag hitag2 read` executed
- Stays on for ~5-6ms (timeslot duration)
- Then turns off

**If LED doesn't light:**
- Timeslot not being granted
- Try disconnecting BLE
- Check timeslot request succeeded

### 2. Check Serial Debug Logs

**Enable logging:**
- Connect to serial console
- Baud: 115200
- Watch for NRF_LOG output

**Expected log messages:**
```
Hitag2 RTF protocol with correct BPLM encoding starting...
Transmitting START_AUTH with BPLM encoding...
START_AUTH transmission complete
```

**If no logs appear:**
- Firmware not properly flashed
- Logging disabled in build
- Wrong serial port/baud

### 3. Proxmark3 Setup

**Verify PM3 can detect field:**
```bash
# On Proxmark3
lf tune

# On ChameleonUltra
lf hitag hitag2 read

# PM3 should show voltage change
```

**PM3 positioning:**
- Distance: 1-5cm from Chameleon
- Too close: Signal saturation
- Too far: Signal too weak
- Orientation matters (coil alignment)

**PM3 firmware:**
- Update to latest RRG firmware
- Check: `hw version`
- Verify LF antenna connected

### 4. Test Basic Field Toggle

Add this test code to `hitag2_timeslot_callback()`:

```c
// Slow toggle test - PM3 should definitely see this
NRF_LOG_INFO("Starting slow toggle test...");
for (int i = 0; i < 10; i++) {
    start_lf_125khz_radio();
    bsp_delay_us(10000);  // 10ms ON
    stop_lf_125khz_radio();
    bsp_delay_us(10000);  // 10ms OFF
}
NRF_LOG_INFO("Slow toggle test complete");
```

**PM3 should detect:**
- Field turning on/off
- 10 distinct pulses
- If not detected: Hardware issue

### 5. Increase Pulse Duration

Make pulse easier to detect:

In `lf_hitag2_data.h`:
```c
// Original (per spec):
#define HITAG2_BPLM_PULSE_US     48
#define HITAG2_BPLM_BIT0_HIGH_US 112
#define HITAG2_BPLM_BIT1_HIGH_US 192

// Test with longer pulse:
#define HITAG2_BPLM_PULSE_US     80   // Longer OFF
#define HITAG2_BPLM_BIT0_HIGH_US 80   // Adjusted (80+80=160)
#define HITAG2_BPLM_BIT1_HIGH_US 160  // Adjusted (80+160=240)
```

Maintains correct total timing but with longer, more detectable pulse.

### 6. Check Antenna Connection

**Physical inspection:**
- LF antenna wire connected?
- Not HF antenna by mistake?
- No broken connections?

**Test with other LF protocols:**
```bash
# Try EM410x (known working)
lf em 410x read

# If this works, antenna is fine
# If this fails, antenna problem
```

### 7. Verify Compilation

**Check build output:**
```bash
cd firmware/application
make clean
make 2>&1 | grep -i hitag

# Should see:
# Compiling hitag.c
# Compiling lf_hitag2_data.c
# No errors
```

**Verify flash:**
- Note firmware size
- Flash to device
- Verify flash completed successfully

## Common Issues

### Issue 1: BLE Interference

**Symptom:** LED doesn't light, or flickers briefly
**Cause:** Timeslot not granted due to BLE activity
**Solution:**
- Disconnect BLE before LF operations
- Use USB connection for testing
- Check timeslot_request return value

### Issue 2: Distance/Positioning

**Symptom:** PM3 sees nothing, but LED lights
**Cause:** Poor coil coupling
**Solution:**
- Try different distances (1-5cm)
- Rotate Chameleon 90° (coil alignment)
- Use PM3 antenna tuning (`hw tune`) to find best position

### Issue 3: PWM Not Starting

**Symptom:** No field detected by PM3 or scope
**Cause:** PWM initialization failed
**Solution:**
- Check `lf_125khz_radio_init()` called
- Verify PWM pins configured
- Check for initialization errors in log

### Issue 4: Timing Too Fast

**Symptom:** PM3 sees "noise" but no auth attempts
**Cause:** Transitions too fast for PM3 decoder
**Solution:**
- Increase pulse duration (see step 5)
- Add more settling time
- Test with scope to verify actual timing

## Alternative Testing Methods

### Using Logic Analyzer

**Connect to:**
- LF_ANT_DRIVER pin
- Ground

**Expected waveform:**
- 125kHz square wave during "ON"
- Low/high-Z during "OFF"
- Clear transitions at pulse boundaries

**Measurements:**
- Bit 0: 160μs ± 10%
- Bit 1: 240μs ± 10%
- Pulse: 48μs ± 10%

### Using Oscilloscope

**Probe LF antenna:**
- 10x probe recommended
- AC coupling
- 200kHz+ bandwidth

**Expected signal:**
- 125kHz carrier when ON
- Quiet when OFF
- Pulse duration measurable

### Using Software-Defined Radio (SDR)

**Setup:**
- Tune to 125kHz (or baseband)
- AM demodulation
- Should see carrier on/off

## Code Quality Verification

### Timing Audit Checklist

- [x] Bit 0 = 160μs (48 + 112)
- [x] Bit 1 = 240μs (48 + 192)
- [x] START_AUTH = 5 bits (11000)
- [x] PWM settling = 15μs accounted
- [x] Timeslot duration = 10ms (sufficient)
- [x] Field powered for tag = Yes
- [x] Matches Proxmark3 pattern = Yes

### Hardware Compliance Checklist

- [x] NRF52 PWM spec followed
- [x] Timeslot API used correctly
- [x] GPIO interrupts configured
- [x] Circular buffer for reception
- [x] Manchester decoder ready

### Code Pattern Compliance

- [x] Follows T55xx timeslot pattern
- [x] Follows EM410x reception pattern
- [x] Follows Viking structure
- [x] Consistent with codebase style

## Expected vs. Actual

### What Should Happen

1. User runs: `lf hitag hitag2 read`
2. LED lights up (field ON)
3. Timeslot granted (10ms)
4. START_AUTH transmitted (~3ms into timeslot)
5. PM3 sniff detects: "Auth attempts... 1"
6. LED turns off
7. Result: "LF tag not found" (expected, no tag present)

### What User Reports

1. User runs: `lf hitag hitag2 read`
2. LED lights up briefly (good)
3. PM3 sniff shows: "Auth attempts... 0" (bad)
4. Result: "LF tag not found"

### Gap Analysis

**LED lights:** Timeslot granted ✓
**PM3 sees nothing:** Signal issue ❌

**Possible causes:**
1. Field not actually modulating (PWM issue)
2. Modulation too fast for PM3 decoder
3. PM3 too far / misaligned
4. PM3 not in correct mode
5. Interference masking signal

## Recommended Action Plan

1. **Verify basic field detection** (lf tune test)
2. **Check serial logs** (verify code execution)
3. **Try slow toggle test** (confirm PM3 sees on/off)
4. **Test with increased pulse** (make signal obvious)
5. **Use scope/analyzer** (verify actual waveform)
6. **Check antenna** (try other LF protocols)

If all steps pass except step 3 (slow toggle), hardware issue likely.
If step 3 passes but BPLM not detected, timing/encoding issue.

## Support

For further debugging:
1. Capture serial log output
2. Record PM3 sniff session
3. Note LED behavior
4. Document test conditions (distance, orientation)
5. Report which diagnostic steps passed/failed

This will help identify whether issue is:
- Hardware (antenna, connections)
- Software (timing, encoding)
- Setup (positioning, interference)
- Environmental (other devices, metal)
