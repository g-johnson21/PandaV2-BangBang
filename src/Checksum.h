#pragma once
#include <stddef.h>
#include <stdint.h>

// Byte-wise XOR checksum used by the EEPROM blocks (BB config, PT tare).
// Chain calls through `seed` to cover several regions.
inline uint8_t xorChecksum(const void* data, size_t len, uint8_t seed = 0) {
    const uint8_t* p = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < len; i++) seed ^= p[i];
    return seed;
}
