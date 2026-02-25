#pragma once

#include <cstdint>

// Bitmask flags
constexpr uint8_t RESET_LEFT  = 1 << 0;
constexpr uint8_t RESET_RIGHT = 1 << 1;
constexpr uint8_t RESET_BACK  = 1 << 2;
// (Optional if you add a front sensor later)
// constexpr uint8_t RESET_FRONT = 1 << 3;

/**
 * Resets chassis odom X and/or Y using distance sensors to field walls.
 *
 * @param flags bitmask of RESET_LEFT/RESET_RIGHT/RESET_BACK
 * @param min_mm ignore readings below this (noise / invalid)
 * @param max_mm ignore readings above this (out of range / invalid)
 */
void distanceReset(uint8_t flags, double min_mm = 20.0, double max_mm = 2000.0);