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

### ✅ Implemented
- Basic protocol framework following ChameleonUltra architecture
- Tag type definitions and registration
- PWM modulation for Manchester-encoded tag responses
- T55xx configuration for Hitag2 tags
- Factory data initialization with default UIDs
- Data load/save callbacks for flash persistence
- Proper null pointer checks and error handling
- Named constants for timing and PWM parameters

### ⚠️ Partial/Simplified
- BPLM decoder for downstream communication
  - Currently uses simplified decoding
  - Production use requires proper edge timing analysis
  - Should count transitions per bit period accurately

### ❌ Not Yet Implemented
- **Crypto Authentication**: Hitag2 uses a proprietary 48-bit stream cipher
  - Challenge-response authentication
  - Requires implementation of Hitag2 crypto algorithms
  - See Proxmark3's `hitag2_crypto.c` for reference
- **Password Mode**: Simple password-based authentication
- **State Machine**: Full protocol state machine for different authentication modes
- **Multi-block Read/Write**: Currently only supports UID emulation

## File Structure

```
firmware/application/src/rfid/nfctag/lf/protocols/
├── hitag.h              # Protocol declarations and constants
└── hitag.c              # Implementation (codec, modulator, decoder)

firmware/application/src/rfid/nfctag/lf/
├── lf_tag_em.h          # LF tag handler declarations (updated)
└── lf_tag_em.c          # LF tag handlers (updated)

firmware/application/src/rfid/nfctag/
├── tag_base_type.h      # Tag type enums (updated)
└── tag_emulation.c      # Protocol registration (updated)
```

## Usage

### Factory Default Data
When a slot is initialized with Hitag2 type, it gets a default UID:
- Default UID: `01 02 03 04`

### Customization
To use a specific Hitag2 UID, modify the tag data in the slot:
1. Set slot type to TAG_TYPE_HITAG2 or TAG_TYPE_HITAG2_PAXTON
2. Write the 4-byte UID to the slot data buffer
3. The firmware will automatically modulate the UID using Manchester encoding

## Technical References

1. **Proxmark3 Implementation**
   - https://github.com/RfidResearchGroup/proxmark3/tree/master/armsrc
   - Files: `hitag2.c`, `hitag_common.c`, `hitag2_crypto.c`

2. **Hitag2 Protocol Documentation**
   - http://www.proxmark.org/files/Documents/125%20kHz%20-%20Hitag/HT2protocol.pdf

3. **T55xx Configuration**
   - Uses Manchester encoding for upstream
   - RF/50 data rate
   - Configuration: `T5577_BITRATE_RF_50 | T5577_MODULATION_MANCHESTER`

## Limitations and Future Work

### Current Limitations
1. **No Crypto Support**: Cannot authenticate with readers requiring crypto mode
2. **Simplified BPLM Decoder**: May not work reliably with all readers
3. **UID-Only Emulation**: Only emulates UID response, not full tag memory
4. **No Read/Write Commands**: Does not support block read/write operations
5. **Hardware Testing Required**: Implementation needs validation on actual hardware

### Future Enhancements
1. Implement Hitag2 crypto authentication
   - Port crypto algorithms from Proxmark3
   - Add key storage and management
2. Improve BPLM decoder with proper edge detection
3. Add support for password mode authentication
4. Implement full state machine for different modes
5. Add multi-block read/write support
6. Add support for Hitag1 and HitagS variants

## Testing

⚠️ **Hardware testing required**: This implementation has not been tested on actual ChameleonUltra hardware. Users should:

1. Test with known-good Hitag2 tags first
2. Verify timing with oscilloscope if available
3. Test with various reader types
4. Report any issues or required timing adjustments

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
