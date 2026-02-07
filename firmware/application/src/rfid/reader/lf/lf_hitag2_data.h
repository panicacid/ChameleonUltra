#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

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
