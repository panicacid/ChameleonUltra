# Hitag2 Receiver Analysis: Why We Can't Hear the Card

## User's Observation (CRITICAL)
**"I can clearly see on the scope that we're blasting out our five bits, 11000 then a few µs later the carrier wave is up and down long and short, the card is 100% replying to us and we're just completely blind to it (or deaf as the case may be)."**

This confirms:
1. ✅ Transmitter working perfectly (5 bits: 11000 visible)
2. ✅ Card IS responding (Manchester modulation visible) 
3. ✅ Response timing seems correct ("a few µs later")
4. ❌ Receiver not detecting anything

## Current Implementation Analysis

### Timeslot Flow
```c
hitag2_read():
  1. init_hitag2_hw() - Enable GPIO interrupts
  2. request_timeslot(10000µs, hitag2_timeslot_callback)
     └─> hitag2_timeslot_callback():
           - start_lf_125khz_radio()          // Field ON
           - delay 2504µs (powerup)
           - delay 464µs (start_auth window)
           - hitag2_send_start_auth()         // Send 5 bits + terminator
             └─> Field ends ON after terminator
  3. delay 5000µs (wait for response)
  4. Process circular buffer
```

### Critical Timing Question

**When does timeslot END vs when does card RESPOND?**

Current timeslot: **10ms (10,000µs)**

Time used in timeslot:
- Powerup: 2,504µs
- Start auth window: 464µs
- Transmission: ~1,000µs (5 bits + terminator)
- **Total: ~4,000µs**

**Remaining: ~6,000µs in timeslot**

Card response delay (from scope): "a few µs later"
- If "few" = 10-100µs, response is INSIDE timeslot ✓
- If "few" = 200-500µs, response is INSIDE timeslot ✓
- Should be detectable!

BUT: **Are GPIO interrupts DISABLED during timeslot?**

## THE CRITICAL ISSUE: GPIO Interrupts During Timeslot

### Timeslot Behavior (from timeslot.c)

```c
static void t55xx_soc_evt_handler(uint32_t evt_id, void *p_context)
```

The timeslot callback runs with **SOFTDEVICE CONTROL**. During timeslot:
- Radio is under timeslot control
- Some interrupts may be blocked
- GPIO interrupts might be disabled!

### Comparison with T55xx (Working)

**T55xx is WRITE-ONLY:**
```c
void t55xx_timeslot_callback() {
    // Send data
    // NO LISTENING - field stays on, we're done
}
```

**Hitag2 needs RTF (Read Tag First):**
```c
void hitag2_timeslot_callback() {
    // Send START_AUTH
    // Field ON
    // ??? How to listen while in timeslot?
}
```

## PROBLEM 1: Listening Inside vs Outside Timeslot

### Current Approach
```
Timeslot (10ms):
  [Start field] [Powerup 2.5ms] [Window 464µs] [Send 1ms] [???] 
  ^                                                         ^
  |                                                         |
GPIO interrupts working here?                    Still in timeslot!
```

**After timeslot ends:**
```c
bsp_delay_us(5000);  // Wait AFTER timeslot
// Process buffer
```

### Question: Is card responding INSIDE or AFTER timeslot?

**From scope:** "a few µs later" suggests response is IMMEDIATE (inside timeslot)

**But:** If GPIO interrupts disabled during timeslot, we won't capture edges!

## PROBLEM 2: Field State During Listening

### Current Code After Transmission
```c
// In hitag2_send_start_auth():
stop_lf_125khz_radio();
bsp_delay_us(HITAG2_BPLM_PULSE_US);  // Terminating pulse
start_lf_125khz_radio();  // Field ON
// Callback ends, return from timeslot
```

**Field is ON when we return from timeslot callback.**

### After Timeslot
```c
// In hitag2_read():
request_timeslot(10000, hitag2_timeslot_callback);
// Timeslot callback completes
NRF_LOG_INFO("START_AUTH transmitted");

// Field is now ON (left on by timeslot callback)
bsp_delay_us(5000);  // Wait for response
// Try to decode
```

**Question:** Is field still ON? Or does timeslot cleanup turn it off?

## Reference Implementation Analysis

### Proxmark3 hitag2.c (Lines 300-400)

```c
void ht2_reader_send_frame(byte_t* frame, size_t nbits, ...) {
    // Send bits with BPLM
    // Last bit ends with field ON
}

static int GetIso15693AnswerFromTag(byte_t* response, ...) {
    // CONTINUOUS listening immediately after transmission
    // No delay between send and receive
    // Uses analog signal processing, not GPIO edges
}

// Pattern:
SendFrame();
AT91C_BASE_SSC->SSC_CR = AT91C_SSC_RXEN;  // Enable RX immediately!
GetResponseFromTag();  // Start listening NOW
```

**Key difference:** Proxmark3 enables receiver IMMEDIATELY after transmission, no waiting!

### RFIDler hitag.c (Lines 100-200)

```c
void hitag_reader_send_frame(...) {
    // Send BPLM bits
    // Last bit ends with field ON
}

void hitag_get_response() {
    // Continuous ADC sampling during field ON
    // Uses analog comparator, always listening
}

// Pattern:
hitag_send_frame();
// NO DELAY - already listening via analog input
hitag_decode_manchester();
```

**Key difference:** RFIDler uses analog input that's ALWAYS listening, no enable/disable!

## Problem Summary

### Issue 1: GPIO Interrupts During Timeslot
**Hypothesis:** GPIO interrupts might be disabled during timeslot, so we don't capture edges when card responds immediately after transmission.

**Test:** Check if any edges are captured at all (debug logs should show edge count).

### Issue 2: Timing of Listener Enable
**Current:** 
```
[Timeslot: Send START_AUTH] → [End timeslot] → [Delay 5ms] → [Process buffer]
```

**Should be:**
```
[Timeslot: Send START_AUTH] → [ALREADY listening] → [Process edges in real-time]
```

### Issue 3: Field State Uncertainty
**Question:** Is field still ON after timeslot returns? We assume yes, but need to verify.

## Proposed Solutions

### Solution 1: Enable GPIO Before Timeslot
```c
// BEFORE timeslot
init_hitag2_hw();  // Enable GPIO interrupts

// Request timeslot
request_timeslot(10000, hitag2_timeslot_callback);

// GPIO interrupts should capture edges during timeslot
```

**Status:** Already doing this! But are interrupts actually working during timeslot?

### Solution 2: Extend Timeslot to Cover Response
```c
// Increase timeslot to 15ms instead of 10ms
request_timeslot(15000, hitag2_timeslot_callback);

// In callback:
hitag2_send_start_auth();
// Field ON
// Wait for response INSIDE timeslot
bsp_delay_us(5000);  // Response window inside timeslot
// Return - edges should be captured
```

**Advantage:** Ensures GPIO interrupts active during entire response window.

### Solution 3: Don't Use Timeslot for Listening
```c
// Start field OUTSIDE timeslot
start_lf_125khz_radio();
bsp_delay_us(2504);  // Powerup

// Use timeslot ONLY for precision transmission
request_timeslot(5000, hitag2_transmit_only_callback);
// Callback sends START_AUTH and returns

// Field stays ON
// GPIO interrupts capture response
bsp_delay_us(5000);
// Process buffer
```

**Advantage:** GPIO interrupts definitely active outside timeslot.

### Solution 4: Start Listening Before Transmission Ends
```c
// In timeslot callback, before last bit:
// Enable edge capture
init_hitag2_hw();  // May need to call from inside timeslot?

// Send START_AUTH
// Edges captured immediately as they occur
```

## Diagnostic Questions for User

### 1. Edge Capture
**Check debug logs:** Are ANY edges being captured?
```
[DEBUG] Edge #0: interval=XX
[DEBUG] Edge #1: interval=XX
```

- **If 0 edges:** GPIO interrupts not working (timeslot issue or hardware)
- **If >0 edges:** Decoder/threshold issue

### 2. Timing
**On scope:** Measure delay between:
- End of terminating pulse (field goes ON)
- Start of card response (first Manchester edge)

Is it:
- < 100µs? (Very fast, inside timeslot)
- 100-500µs? (Still inside timeslot)  
- > 1ms? (Outside timeslot, should be captured)

### 3. Field State
**Check with scope:** After START_AUTH transmission:
- Does field stay continuously ON?
- Or does it drop off at some point?

## Most Likely Root Cause

Based on analysis, **most likely issue is:**

**GPIO interrupts are being blocked or not firing during/immediately after timeslot.**

The card responds immediately (as seen on scope), but we're either:
1. Still in timeslot with interrupts disabled
2. Not listening yet (waiting outside timeslot)
3. Field drops between transmission and listening

## Recommended Fix Priority

### Priority 1: Verify Edge Capture
- Check debug logs for edge count
- Confirms if GPIO working at all

### Priority 2: Extend Timeslot
- Include response window IN timeslot
- Increase from 10ms to 20ms
- Add delay in callback after transmission

### Priority 3: Move Field Control Outside Timeslot
- Use timeslot ONLY for transmission
- Listen outside timeslot where GPIO definitely works

### Priority 4: Check Field Continuity  
- Verify field stays ON after timeslot
- Check with scope or multimeter

## Next Steps

1. **User shares debug log** showing edge capture count
2. **User measures timing** on scope (end of TX to start of RX)
3. **Try extended timeslot** (20ms with response window inside)
4. **Try no-timeslot approach** (use timeslot only for TX precision)

The receiver code itself is correct (proven by EM410x working). The issue is **timing coordination** between transmission, timeslot, and listening window.
