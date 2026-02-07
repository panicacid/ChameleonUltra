#pragma once

#include "protocols.h"

// Hitag/Hitag2 tag ID sizes
#define HITAG_UID_SIZE 4
#define HITAG_BLOCK_SIZE 4
#define HITAG2_MAX_BLOCKS 8
#define HITAG_PASSWORD_SIZE 4
#define HITAG_CRYPTOKEY_SIZE 6

// Basic Hitag2 configuration
#define HITAG2_CONFIG_BLOCK 3

// Hitag protocols
extern const protocol hitag2;
extern const protocol hitag2_paxton;

extern const protocol* hitag_protocols[];
extern size_t hitag_protocols_size;

// T55xx writer for Hitag
uint8_t hitag2_t55xx_writer(uint8_t* uid, uint32_t* blks);
