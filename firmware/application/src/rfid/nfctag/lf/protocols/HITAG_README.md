# Hitag/Hitag2/Paxton Hitag2 Implementation

## Overview

This implementation adds support for Hitag, Hitag2, and Paxton Hitag2 protocols to the ChameleonUltra firmware. These are 125kHz Reader-Talk-First (RTF) RFID protocols commonly used in access control systems.

## Protocol Details

### Encoding Schemes

Based on the Proxmark3 implementation (https://github.com/RfidResearchGroup/proxmark3/blob/master/armsrc/hitag2.c):

**Downstream (Reader → Tag): BPLM (Binary Pulse Length Modulation)**
- Each bit starts with a fixed PULSE (field OFF/dropped)
- Followed by field ON for variable duration
- Bit value encoded in the length of the ON period
- **NOT** bi-phase mark (transition-based) or gap modulation (T55xx style)
- Pattern: |___PULSE___|------ON------|

**Timing from Proxmark3:**
```c
#define HITAG_T_LOW  6   // Pulse duration: 6 Tc = 48μs (field OFF)
#define HITAG_T_0    20  // Bit 0 total: 20 Tc = 160μs
#define HITAG_T_1    30  // Bit 1 total: 30 Tc = 240μs

Bit 0: OFF(48μs) + ON(112μs) = 160μs
Bit 1: OFF(48μs) + ON(192μs) = 240μs
```

**Upstream (Tag → Reader): Manchester Encoding (IEEE 802.3)**
- Logic 0: High-to-low transition in middle of bit period
- Logic 1: Low-to-high transition in middle of bit period

### Timing
- Data Rate: RF/50 (50 RF cycles per bit period)
- Carrier Frequency: 125kHz (Tc = 8μs)
- UID Size: 4 bytes

### Supported Tag Types

1. **TAG_TYPE_HITAG2 (400)** - Standard Hitag2
   - Used in general access control and authentication
   - Supports password and crypto authentication modes

2. **TAG_TYPE_HITAG2_PAXTON (402)** - Paxton Hitag2 Variant
   - Specialized variant used in Paxton access control systems
   - Same underlying protocol with vendor-specific implementation

## Implementation Status

### Fully Implemented ✅

**BPLM (Binary Pulse Length Modulation) Encoding:**
- Correct OFF-then-ON pattern matching Proxmark3 implementation
- Each bit starts with 48μs PULSE (field OFF)
- Followed by variable ON time (112μs for bit 0, 192μs for bit 1)
- Total bit timing: 160μs (bit 0), 240μs (bit 1) per Hitag2 spec
- START_AUTH command transmission (5 bits: 11000)
- **Timeslot API integration** for precision timing without BLE interference
- **Timing verified** against Proxmark3 hitag2.c implementation
- **Should be detectable** by Proxmark3 sniffer (ready for hardware testing)

**Reader Implementation (lf_hitag2_data.c):**
- RTF protocol sequence with proper timing
- Powerup wait (2.5ms)
- START_AUTH window timing (464μs)
- BPLM bit transmission
- GPIO interrupt-based response reception
- Manchester decoder for 32-bit UID

**CLI Commands:**
- `lf hitag hitag2 read` - Scan and display Hitag2 UID
- `lf hitag hitag2 write --id <hex>` - Write UID to T55xx tag
- `lf hitag hitag2 econfig --id <hex>` - Configure emulation

**Firmware Command Handlers:**
- HITAG2_SCAN (3007) - Read tag UID
- HITAG2_WRITE_TO_T55XX (3008) - Write to T55xx
- HITAG2_SET_EMU_ID (5006) - Set emulation UID
- HITAG2_GET_EMU_ID (5007) - Get emulation UID

**Infrastructure:**
- PWM modulation for Manchester-encoded tag responses
- T55xx configuration for Hitag2 tags (Manchester/RF50)
- Factory data initialization with default UIDs
- Data load/save callbacks for flash persistence
- Proper error handling and null checks
- Named constants for timing parameters

### Requires Hardware Testing ⚠️

- **Tag UID Reading**: BPLM transmission should now be detectable (encoding fixed), actual UID reading needs validation
- **Manchester Response Decoding**: Decoder implemented but needs testing with real tag responses
- **Timing Adjustments**: May require fine-tuning based on real-world tag behavior
- **Different Tag Variants**: Needs testing with both standard Hitag2 and Paxton variants

### Not Yet Implemented ❌

- **Crypto Authentication**: Hitag2 uses a proprietary 48-bit stream cipher
  - Challenge-response authentication
  - Requires implementation of Hitag2 crypto algorithms
  - See Proxmark3's `hitag2_crypto.c` for reference
- **Password Authentication**: Password mode (32-bit password)
  - Simpler than crypto, but still requires protocol implementation
- **Block Read/Write**: Reading/writing data blocks beyond UID
  - Requires implementing full command set
- **Advanced Modes**: Public mode variants, test modes, etc.

## File Structure

```
firmware/application/src/
├── rfid/nfctag/
│   ├── tag_base_type.h           [Modified] Added TAG_TYPE_HITAG2/PAXTON enums
│   ├── tag_emulation.c           [Modified] Registered Hitag protocols
│   └── lf/
│       ├── lf_tag_em.c           [Modified] Added Hitag load/save/factory handlers
│       ├── lf_tag_em.h           [Modified] Added Hitag function declarations
│       └── protocols/
│           ├── hitag.c           [Created] Protocol implementation (codec, modulator)
│           ├── hitag.h           [Created] Protocol header
│           ├── t55xx.h           [Modified] Added T5577_HITAG2_CONFIG
│           └── HITAG_README.md   [Created] This documentation
├── rfid/reader/lf/
│   ├── lf_reader_main.c          [Modified] Added scan_hitag2() and write functions
│   ├── lf_reader_main.h          [Modified] Added function declarations
│   ├── lf_reader_data.h          [Modified] Added hitag2_read() declaration
│   ├── lf_hitag2_data.c          [Created] Hitag2 reader with BPLM and timeslot
│   └── lf_hitag2_data.h          [Created] Reader header with timing constants
├── app_cmd.c                     [Modified] Added command processors
├── data_cmd.h                    [Modified] Added command definitions
├── utils/
│   ├── timeslot.c                [Existing] Radio timeslot API
│   └── timeslot.h                [Existing] Timeslot header
└── Makefile                      [Modified] Added hitag.c and lf_hitag2_data.c

software/script/
├── chameleon_enum.py             [Modified] Added Hitag commands and tag types
├── chameleon_cli_unit.py         [Modified] Added CLI command structure
└── chameleon_cmd.py              [Modified] Added command implementations
```

## Usage

### CLI Commands

**Scan Hitag2 Tag:**
```bash
chameleon lf hitag hitag2 read
```
- Activates reader mode
- Sends START_AUTH with BPLM encoding
- Displays UID if tag responds

**Write to T55xx:**
```bash
chameleon lf hitag hitag2 write --id 01020304
```
- Configures T55xx as Hitag2 tag
- Uses Manchester/RF50 encoding
- Writes UID to block 0

**Configure Emulation:**
```bash
chameleon lf hitag hitag2 econfig --id 01020304
```
- Sets emulated UID for current slot
- Works in emulation mode

### Protocol Behavior

**Reader Mode:**
1. Field powers on (LED lights)
2. Tag powerup delay (2.5ms)
3. START_AUTH window (464μs)
4. BPLM transmission with timeslot
5. Waits for Manchester response
6. Decodes 32-bit UID

**Emulation Mode:**
- Not yet fully implemented
- Infrastructure in place
- Needs PWM modulator completion

## Testing

**Implementation Verified:**

The BPLM encoding has been corrected to match Proxmark3's proven implementation:

**Encoding Verified:**
- ✅ OFF-then-ON pattern (not ON-then-OFF)
- ✅ 48μs pulse duration (not 20μs gap)
- ✅ 112μs/192μs ON times (not 140μs/100μs)
- ✅ Timeslot API integrated
- ✅ Code matches Proxmark3 hitag2_reader_send_bit()

**Ready for Hardware Testing:**

```bash
# On Proxmark3 (sniffer)
[usb] pm3 --> lf hitag sniff
[=] Press pm3 button to abort sniffing

# On ChameleonUltra
chameleon lf hitag hitag2 read

# Expected on Proxmark3:
[#] Auth attempts... 1  (should be detected now)
[=] Done!

# View trace
[usb] pm3 --> lf hitag list
# Should show START_AUTH command with correct BPLM timing
```

**Expected Results:**
- ✅ LED lights up during scan (field active)
- ✅ Proxmark3 detects auth attempts (was 0, should be 1+)
- ✅ START_AUTH command visible in trace
- ⚠️ Tag response depends on Manchester decoder (needs testing)

**With Real Tags:**
```bash
chameleon lf hitag hitag2 read
# Should display: "Hitag2 UID: XX XX XX XX" if successful
# Or: "LF tag not found" if tag doesn't respond
```

**Debugging:**
- Check LED activity (should light during scan)
- Use Proxmark3 sniffer to verify BPLM transmission
- Verify START_AUTH command in trace
- Check timing with logic analyzer if available

## Technical Implementation

**BPLM Encoding (Binary Pulse Length Modulation):**

Correct implementation matching Proxmark3 hitag2.c:

```c
// Each bit: PULSE (OFF) then ON
// Pattern: |___PULSE___|------ON------|

// Bit 0 (160μs total):
stop_lf_125khz_radio();          // Pulse start (OFF)
bsp_delay_us(48);                // Pulse duration (6 Tc)
start_lf_125khz_radio();         // Field ON
bsp_delay_us(112);               // ON duration (14 Tc)
// Total: 48 + 112 = 160μs ✓

// Bit 1 (240μs total):
stop_lf_125khz_radio();          // Pulse start (OFF)
bsp_delay_us(48);                // Pulse duration (6 Tc)
start_lf_125khz_radio();         // Field ON
bsp_delay_us(192);               // ON duration (24 Tc)
// Total: 48 + 192 = 240μs ✓
```

**START_AUTH Waveform:**
```
Command: 11000 (5 bits)
Each bit: PULSE (48μs OFF) + ON (112μs or 192μs)

Bit 1: |_____|-----------------|
Bit 1: |_____|-----------------|
Bit 0: |_____|-----------|
Bit 0: |_____|-----------|
Bit 0: |_____|-----------|
```

**Timeslot Integration:**
```c
// Use timeslot API for time-critical transmission
static void hitag2_timeslot_callback(void) {
    bsp_delay_us(2504);  // Powerup wait
    bsp_delay_us(464);   // START_AUTH window
    hitag2_send_start_auth();  // Transmit with BPLM
}

// In hitag2_read():
start_lf_125khz_radio();  // Power tag
request_timeslot(5000, hitag2_timeslot_callback);
```

**Manchester Decoding (Tag Response):**
```c
// Uses GPIO interrupts to capture edge timing
// Circular buffer stores timing data
// hitag2.decoder.feed() processes each edge
// Returns true when 32-bit UID decoded
```

**Hardware Platform:**
- NRF52840 microcontroller
- PWM module for 125kHz carrier generation
- GPIO interrupts for edge detection
- Timeslot API for BLE coexistence

## Limitations

**Current Implementation:**
1. **UID-only**: Only reads/emulates UID, no block data
2. **Public Mode**: No authentication support yet
3. **BPLM Decoder Stub**: Downstream decoding not implemented (not needed for reader mode)
4. **Simplified Implementation**: Basic RTF protocol only

**Hardware Limitations:**
1. **PWM Timing**: NRF52 PWM has latency, using timeslot API compensates
2. **Field Strength**: May vary compared to commercial readers
3. **Timing Precision**: Microsecond-level via bsp_delay_us()

**Future Enhancements Needed:**
1. Implement Hitag2 cipher (48-bit stream cipher)
2. Add challenge-response authentication
3. Support password mode (32-bit password)
4. Implement block read/write commands
5. Test with various tag manufacturers
6. Optimize timing based on real-world results
7. Add full emulation mode support

## Technical References

**Proxmark3 Implementation:**
- `armsrc/hitag2.c` - Main Hitag2 implementation
- `armsrc/hitag2.h` - Definitions and constants
- `common/hitag2/hitag2_crypto.c` - Crypto implementation
- https://github.com/RfidResearchGroup/proxmark3/tree/master

**Hitag2 Documentation:**
- HT2protocol.pdf - Official protocol specification
- http://www.proxmark.org/files/Documents/125%20kHz%20-%20Hitag/

**ChameleonUltra Implementations Used as Reference:**
- `lf_t55xx_data.c` - T55xx writer for timeslot API usage and field control
- `lf_em410x_data.c` - EM410x reader for Manchester decoding and GPIO interrupts
- `lf_viking_data.c` - Viking reader for similar LF protocol structure
- `timeslot.c/h` - Radio timeslot API for time-critical operations without BLE interference

## Contributing

When contributing to this implementation:

1. **Test with real hardware** - Proxmark3 sniffer and actual tags
2. **Maintain timing accuracy** - Use timeslot API for precision
3. **Follow existing patterns** - Match EM410x/Viking/T55xx style
4. **Document changes** - Update this README
5. **Reference Proxmark3** - Match proven implementations where possible

## License

This implementation follows the ChameleonUltra project license.
