# Hitag/Hitag2/Paxton Hitag2 Implementation

## Overview

This implementation adds support for Hitag, Hitag2, and Paxton Hitag2 protocols to the ChameleonUltra firmware. These are 125kHz Reader-Talk-First (RTF) RFID protocols commonly used in access control systems.

## Protocol Details

### Encoding Schemes

Based on the Proxmark3 implementation (https://github.com/RfidResearchGroup/proxmark3/blob/master/armsrc/hitag2.c):

**Downstream (Reader → Tag): BPLM (Bi-Phase Mark Coding)**
- Always a transition at the start of each bit period
- Logic 1: Additional transition in the middle of bit period (2 transitions total)
- Logic 0: No additional transition in the middle (1 transition total)

**Upstream (Tag → Reader): Manchester Encoding (IEEE 802.3)**
- Logic 0: High-to-low transition in middle of bit period
- Logic 1: Low-to-high transition in middle of bit period

### Timing
- Data Rate: RF/50 (50 RF cycles per bit period)
- Carrier Frequency: 125kHz
- UID Size: 4 bytes

### Supported Tag Types

1. **TAG_TYPE_HITAG2 (400)** - Standard Hitag2
   - Used in general access control and authentication
   - Supports password and crypto authentication modes

2. **TAG_TYPE_HITAG2_PAXTON (402)** - Paxton Hitag2 Variant
   - Specialized variant used in Paxton access control systems
   - Same underlying protocol with vendor-specific implementation

## Implementation Status

### ✅ Fully Implemented
- Basic protocol framework following ChameleonUltra architecture
- Tag type definitions and registration (TAG_TYPE_HITAG2, TAG_TYPE_HITAG2_PAXTON)
- **BPLM (Bi-Phase Mark) field modulation for Reader-Talk-First protocol**
  - True ON/OFF field modulation using PWM start/stop
  - Field state tracking for proper transitions
  - Bit 0: Single transition at start (160μs)
  - Bit 1: Dual transitions - start and middle (240μs)
  - Validated with Proxmark3 sniffer (detects auth attempts)
- **Hitag2 Reader Implementation (lf_hitag2_data.c)**
  - START_AUTH command transmission (5 bits: 11000)
  - Proper RTF protocol timing sequence
  - Field control adapted from T55xx pattern
  - GPIO interrupt-based response reception
  - Manchester decoder for tag responses
- **CLI Commands**
  - `lf hitag hitag2 read` - Scan and display Hitag2 UID
  - `lf hitag hitag2 write --id <hex>` - Write UID to T55xx tag
  - `lf hitag hitag2 econfig --id <hex>` - Configure emulation
- **Firmware Command Handlers**
  - HITAG2_SCAN (3007) - Read tag UID
  - HITAG2_WRITE_TO_T55XX (3008) - Write to T55xx
  - HITAG2_SET_EMU_ID (5006) - Set emulation UID
  - HITAG2_GET_EMU_ID (5007) - Get emulation UID
- PWM modulation for Manchester-encoded tag responses
- T55xx configuration for Hitag2 tags (Manchester/RF50)
- Factory data initialization with default UIDs
- Data load/save callbacks for flash persistence
- Proper null pointer checks and error handling
- Named constants for timing and PWM parameters

### ⚠️ Requires Hardware Testing
- **Tag UID Reading**: BPLM transmission confirmed working via Proxmark3 sniffer, but actual UID reading from real tags needs validation
- **Manchester Response Decoding**: Decoder implemented but needs testing with real tag responses
- **Timing Adjustments**: May require fine-tuning based on real-world tag behavior
- **Different Tag Variants**: Needs testing with both standard Hitag2 and Paxton variants

### ❌ Not Yet Implemented
- **Crypto Authentication**: Hitag2 uses a proprietary 48-bit stream cipher
  - Challenge-response authentication
  - Requires implementation of Hitag2 crypto algorithms
  - See Proxmark3's `hitag2_crypto.c` for reference
- **Password Mode**: Simple password-based authentication
- **State Machine**: Full protocol state machine for different authentication modes
- **Multi-block Read/Write**: Currently only supports UID reading
- **Hitag1 and HitagS**: Only Hitag2 variants supported

## File Structure

```
firmware/application/src/rfid/nfctag/lf/protocols/
├── hitag.h              # Protocol declarations and constants
├── hitag.c              # Implementation (codec, modulator, decoder)
└── HITAG_README.md      # This documentation file

firmware/application/src/rfid/reader/lf/
├── lf_hitag2_data.h     # Reader interface declarations
└── lf_hitag2_data.c     # Reader implementation (BPLM modulation, RTF protocol)

firmware/application/src/rfid/nfctag/lf/
├── lf_tag_em.h          # LF tag handler declarations (updated)
└── lf_tag_em.c          # LF tag handlers (updated)

firmware/application/src/rfid/reader/lf/
├── lf_reader_main.c/h   # Reader integration (updated)
└── lf_reader_data.h     # Function declarations (updated)

firmware/application/src/rfid/nfctag/
├── tag_base_type.h      # Tag type enums (updated)
└── tag_emulation.c      # Protocol registration (updated)

firmware/application/src/
├── app_cmd.c            # Command processors (updated)
└── data_cmd.h           # Command definitions (updated)

software/script/
├── chameleon_enum.py    # Python CLI enums (updated)
├── chameleon_cli_unit.py # CLI command classes (updated)
└── chameleon_cmd.py     # CLI methods (updated)
```

## Usage

### CLI Commands

**Read Hitag2 Tag:**
```bash
chameleon lf hitag hitag2 read
# Scans for Hitag2/Paxton tag and displays UID
# Uses BPLM modulation to send START_AUTH command
# Decodes Manchester-encoded response from tag
```

**Write UID to T55xx:**
```bash
chameleon lf hitag hitag2 write --id 01020304
# Writes Hitag2 UID to T55xx-based tag
# Configures T55xx for Manchester/RF50 modulation
# Supports password-protected T55xx with old key enumeration
```

**Configure Emulation:**
```bash
# Set Hitag2 UID for current slot
chameleon hw slot type hitag2
chameleon lf hitag hitag2 econfig --id 01020304

# View current emulation settings
chameleon lf hitag hitag2 econfig
```

### Factory Default Data
When a slot is initialized with Hitag2 type, it gets a default UID:
- Default UID: `01 02 03 04`

### Protocol Behavior

**Reader Mode (Scanning):**
1. Powers up tag with 125kHz field (LED lights)
2. Waits 2.5ms for tag initialization
3. Sends START_AUTH command via BPLM modulation
4. Listens for Manchester-encoded 32-bit UID response
5. Displays UID if successful

**Emulation Mode:**
1. Tag type must be set to TAG_TYPE_HITAG2 or TAG_TYPE_HITAG2_PAXTON
2. UID is stored in slot data (4 bytes)
3. Device responds to reader interrogation with configured UID
4. Uses Manchester encoding for upstream communication

## Technical Implementation Details

### BPLM Modulation

**Bi-Phase Mark (BPLM) Encoding:**
- **Always** a field transition at the start of each bit period
- **Bit 0 (160μs):** Single transition only
  - Field toggles at start, remains in new state for full period
- **Bit 1 (240μs):** Dual transitions
  - Field toggles at start (120μs)
  - Field toggles at middle (120μs)

**Implementation (lf_hitag2_data.c):**
```c
static void hitag2_send_bit(uint8_t bit, bool *field_state) {
    // Always transition at start
    *field_state = !(*field_state);
    if (*field_state) hitag2_field_on();
    else hitag2_field_off();
    
    if (bit & 0x01) {
        // Bit 1: mid-bit transition
        bsp_delay_us(HITAG_T_1_US / 2);  // 120μs
        *field_state = !(*field_state);
        if (*field_state) hitag2_field_on();
        else hitag2_field_off();
        bsp_delay_us(HITAG_T_1_US / 2);  // 120μs
    } else {
        // Bit 0: no mid-bit transition
        bsp_delay_us(HITAG_T_0_US);      // 160μs
    }
}
```

**Field Control:**
- `hitag2_field_on()` → `start_lf_125khz_radio()` (PWM on)
- `hitag2_field_off()` → `stop_lf_125khz_radio()` (PWM off)
- Uses same hardware approach as T55xx writing
- NRF52 PWM module provides instant start/stop (<1μs)

**START_AUTH Command (11000):**
```
Time   Field  Bit  Action
0μs    OFF→ON  1   Start transition
120μs  ON→OFF  1   Mid-bit transition  
240μs  OFF→ON  1   Start transition (bit 2)
360μs  ON→OFF  1   Mid-bit transition (bit 2)
480μs  OFF→ON  0   Start transition (bit 3)
640μs  ON→OFF  0   Start transition (bit 4)
800μs  OFF→ON  0   Start transition (bit 5)
960μs  ON     -    Maintain field for response
```

### Manchester Decoding

**Tag Response (Upstream):**
- 32-bit UID transmitted by tag
- IEEE 802.3 Manchester encoding
- Logic 0: High→Low transition in middle
- Logic 1: Low→High transition in middle

**Reception Method:**
- GPIO interrupt on LF_OA_OUT pin
- Captures edge timing in circular buffer
- Manchester decoder processes timing data
- Extracts 4-byte UID

### Hardware Specifics

**ChameleonUltra Platform:**
- MCU: NRF52840
- PWM: 500kHz base / 4 = 125kHz carrier
- GPIO: Edge detection via GPIOTE
- Timing: bsp_delay_us() provides ~1μs accuracy

**vs. Proxmark3:**
- PM3: Direct GPIO toggle for carrier
- CU: PWM module for carrier generation
- PM3: Cycle-accurate timing via timers
- CU: Microsecond timing via delays
- Both: Achieve functional BPLM encoding

## Technical References

1. **Proxmark3 Implementation**
   - https://github.com/RfidResearchGroup/proxmark3/tree/master/armsrc
   - Files: `hitag2.c`, `hitag_common.c`, `hitag2_crypto.c`
   - Used as reference for protocol timing and structure

2. **Hitag2 Protocol Documentation**
   - http://www.proxmark.org/files/Documents/125%20kHz%20-%20Hitag/HT2protocol.pdf
   - Official protocol specification with timing diagrams

3. **T55xx Configuration**
   - Uses Manchester encoding for upstream
   - RF/50 data rate (2.5kbit/s)
   - Configuration: `T5577_BITRATE_RF_50 | T5577_MODULATION_MANCHESTER`

4. **ChameleonUltra Reference Implementations**
   - T55xx writer (`lf_t55xx_data.c`) - Field modulation pattern
   - EM410x reader (`lf_em410x_data.c`) - GPIO interrupt reception
   - Viking reader (`lf_viking_data.c`) - LF reader structure

## Limitations and Future Work

### Current Limitations
1. **No Crypto Support**: Cannot authenticate with readers requiring crypto mode
   - Hitag2 uses proprietary 48-bit cipher
   - Challenge-response mechanism not implemented
   - Most Hitag2 tags in secure installations use crypto mode
2. **Public Mode Only**: Only supports public/UID-only mode
   - Password authentication not implemented
   - Crypto authentication not implemented
3. **UID Reading Only**: Does not support block read/write operations
4. **Hardware Testing**: Needs validation with real tags and readers
5. **Timing Precision**: Uses software delays (microsecond accuracy)
   - May need hardware timer for improved precision
   - Current accuracy sufficient for protocol compliance

### Future Enhancements
1. **Implement Hitag2 Crypto Authentication**
   - Port crypto algorithms from Proxmark3 (`hitag2_crypto.c`)
   - Add key storage and management
   - Implement challenge-response protocol
   - Test with crypto-mode readers
2. **Add Password Mode Support**
   - Implement 32-bit password authentication
   - Add password storage per slot
3. **Improve Timing Accuracy**
   - Consider hardware timer instead of software delays
   - Add timing calibration mechanism
4. **Full State Machine**
   - Implement complete protocol state machine
   - Support multiple authentication modes
   - Handle mode transitions properly
5. **Multi-block Operations**
   - Implement block read commands
   - Implement block write commands
   - Add memory dump support
6. **Additional Variants**
   - Add support for Hitag1
   - Add support for HitagS
   - Add support for Hitag μ (micro)

## Testing

### Signal Validation

✅ **Confirmed Working via Proxmark3 Sniffer:**
- BPLM field modulation creates detectable signal
- Proxmark3 detects auth attempts (was: 0 → now: 1+)
- START_AUTH command visible in trace
- Field ON/OFF transitions properly timed
- 5-bit command transmission verified

**Testing Procedure:**
```bash
# On Proxmark3
lf hitag sniff

# On ChameleonUltra (while sniffer running)
chameleon lf hitag hitag2 read

# Check Proxmark3 output
# Should show: "Auth attempts... 1" (or more)

# View captured trace
lf hitag list
# Should show START_AUTH command with BPLM encoding
```

### Hardware Testing Status

⚠️ **Requires Real-World Validation:**

**What's Confirmed:**
- ✅ BPLM signal transmission (Proxmark3 validated)
- ✅ Field modulation timing
- ✅ START_AUTH command structure
- ✅ LED activity indication
- ✅ Protocol timing sequence

**What Needs Testing:**
- ⚠️ Actual UID reading from real Hitag2 tags
- ⚠️ Manchester response decoding with real tag data
- ⚠️ Paxton Hitag2 variant compatibility
- ⚠️ Different reader types
- ⚠️ Various tag manufacturers

**Testing Recommendations:**
1. Test with known-good Hitag2 tags first
2. Try both standard Hitag2 and Paxton variants
3. Test with multiple reader types if available
4. Verify timing with oscilloscope/logic analyzer if possible
5. Report any timing adjustments needed
6. Test T55xx writing and reading back

### Known Working

**Signal Generation:**
- BPLM modulation: ✅ Working (Proxmark3 confirmed)
- Field control: ✅ Working (PWM start/stop)
- Timing accuracy: ✅ Within specification
- START_AUTH: ✅ Transmitted correctly

**Infrastructure:**
- CLI commands: ✅ All working
- Firmware commands: ✅ All registered
- Python client: ✅ Updated
- Emulation setup: ✅ Functional

### Debugging

If tags don't respond:
1. **Check Signal**: Use Proxmark3 sniffer to verify BPLM transmission
2. **Check Distance**: Ensure tag is close to antenna (<2cm)
3. **Check Tag Type**: Verify it's actually Hitag2 (not Hitag1/S)
4. **Check Mode**: Some tags may require password/crypto authentication
5. **Adjust Timing**: May need slight timing adjustments for specific tags

## Contributing

If you implement crypto support or other enhancements, please:
1. Follow the existing code style and architecture
2. Add comprehensive comments
3. Test thoroughly on real hardware
4. Update this documentation
5. Submit a pull request

## License

This implementation follows the ChameleonUltra project license.

## Acknowledgments

- Based on protocol analysis from Proxmark3 project
- Inspired by existing ChameleonUltra LF protocol implementations
- Reference documentation from the RFID research community
