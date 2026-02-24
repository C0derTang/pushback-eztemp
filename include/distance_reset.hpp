// distance_reset.hpp
#pragma once
#include <cstdint>

// Choose which sensors are allowed/required for the reset.
// You can pass any combination of these.
enum DistanceResetMask : uint8_t {
  RESET_NONE  = 0,
  RESET_LEFT  = 1 << 0,
  RESET_RIGHT = 1 << 1,
  RESET_BACK  = 1 << 2,
};

// Returns true if it updated at least ONE component of pose (X and/or Y)
// and (optionally) applied it to odom.
bool distanceReset(uint8_t mask = (RESET_LEFT | RESET_BACK));
/*

// Full reset: left + back (X & Y)
distanceReset(RESET_LEFT | RESET_BACK);

// Full reset: right + back
distanceReset(RESET_RIGHT | RESET_BACK);

// Partial reset: ONLY X using left or right
distanceReset(RESET_LEFT);        // updates X only (if valid), keeps Y from odom
distanceReset(RESET_RIGHT);       // updates X only (if valid), keeps Y from odom
distanceReset(RESET_LEFT | RESET_RIGHT); // uses left if valid else right

// Partial reset: ONLY Y using back
distanceReset(RESET_BACK);        // updates Y only (if valid), keeps X from odom

*/