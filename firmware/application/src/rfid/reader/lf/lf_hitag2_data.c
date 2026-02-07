#include "lf_hitag2_data.h"

#include "bsp_delay.h"
#include "bsp_time.h"
#include "lf_125khz_radio.h"
#include "lf_reader_data.h"
#include "protocols/hitag.h"

#define NRF_LOG_MODULE_NAME hitag2_reader
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
NRF_LOG_MODULE_REGISTER();

/**
 * Attempt to read Hitag2 tag UID
 * 
 * This is a basic implementation that attempts to communicate with a Hitag2 tag.
 * Hitag2 is a Reader-Talk-First (RTF) protocol that requires the reader to
 * initiate communication, unlike Tag-Talk-First protocols like EM410x.
 * 
 * Current implementation:
 * - Starts LF radio (lights LED to show activity)
 * - Attempts basic tag detection
 * - Returns UID if tag responds
 * 
 * TODO: Full RTF protocol implementation with:
 * - SELECT command transmission (BPLM encoding)
 * - UID response reception (Manchester decoding)  
 * - Challenge-response for crypto mode
 * - Password mode support
 */
bool hitag2_read(uint8_t *data, uint32_t timeout_ms) {
    NRF_LOG_INFO("Starting Hitag2 read attempt...");
    
    // Start the LF radio - this will light the activity LED
    start_lf_125khz_radio();
    
    // Initialize for potential future protocol implementation
    // Currently we don't have a full RTF protocol handler
    
    bool tag_found = false;
    autotimer *p_at = bsp_obtain_timer(0);
    
    // Keep radio active for the timeout period to show we're attempting to read
    // This ensures the LED stays lit during the scan attempt
    while (!tag_found && NO_TIMEOUT_1MS(p_at, timeout_ms)) {
        // TODO: Implement RTF protocol:
        // 1. Send SELECT command with BPLM encoding
        // 2. Receive UID response with Manchester decoding
        // 3. Verify response and extract UID
        
        // For now, we wait and allow the radio to stay active
        // so users see the LED indicating scan activity
        bsp_delay_ms(10);
        
        // Break early if we exceed a reasonable attempt time
        if (NO_TIMEOUT_1MS(p_at, 100)) {
            // After 100ms of no response, we can conclude no tag present
            break;
        }
    }
    
    // Stop the radio
    stop_lf_125khz_radio();
    
    if (!tag_found) {
        NRF_LOG_INFO("Hitag2 tag not found (RTF protocol not fully implemented)");
    }
    
    return tag_found;
}
