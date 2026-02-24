#pragma once
#include <cstdint>

namespace rollers {

enum class Mode : uint8_t {
  Stop,
  Store,
  Low,
  Mid,
  High
};

// Initialize internal state (optional, but nice)
void init(int default_voltage = 12000);

// Set desired mode (can be called from auton or opcontrol)
void set_mode(Mode m);

// Get current desired mode
Mode get_mode();

// Set voltage cap (12,000 typical)
void set_voltage(int v);

// Call this repeatedly (e.g., every 20–30ms) to apply motors/pneumatics
// This preserves your current "frame ramp" behavior.
void update();

// Convenience: stop now
inline void stop() { set_mode(Mode::Stop); }

}  // namespace rollers