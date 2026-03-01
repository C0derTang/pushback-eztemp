#include "rollers.hpp"
#include "subsystems.hpp"  // intake, indexer, roller, hood, mid

namespace rollers {

static Mode g_mode = Mode::Stop;
static int  g_voltage = 12000;
static int  g_frame = 0;

void init(int default_voltage) {
  g_voltage = default_voltage;
  g_mode = Mode::Stop;
  g_frame = 0;
}

void set_mode(Mode m) { g_mode = m; }
Mode get_mode() { return g_mode; }

void set_voltage(int v) { g_voltage = v; }

static void apply_direct(Mode m, int v) {
  int intk = 1, idx = 1, rlr = 1, hd = 0, mps = 0;

  if (m == Mode::Store) {
    rlr = 0;                 // store: stop roller
  } else if (m == Mode::Low) {
    intk = idx = rlr = -1;   // low: reverse all
  } else if (m == Mode::Mid) {
    rlr = -1;                // mid: reverse roller only
    mps = 1;                 // mid piston on
  } else if (m == Mode::High) {
    hd = 1;                  // hood on
  } else {                   // stop
    intk = idx = rlr = 0;
  }

  intake.move_voltage(v * intk);
  indexer.move_voltage(v * idx);
  roller.move_voltage(v * rlr);
  hood.set_value(hd);
  mid.set_value(mps);
}

void update() {
  // Preserve your original logic:
  // - frame resets unless high or mid
  // - on mid/high, briefly run low at 9000 before switching to mode at velo
  if (g_mode != Mode::High && g_mode != Mode::Mid) g_frame = 0;
  else ++g_frame;

  const bool warmup_mid  = (g_mode == Mode::Mid  && g_frame < 7);
  const bool warmup_high = (g_mode == Mode::High && g_frame < 3);

  if (warmup_mid || warmup_high) {
    apply_direct(Mode::Low, 9000);
  } else if (g_mode == Mode::High || g_mode == Mode::Mid) {
    apply_direct(g_mode, g_voltage);
  } else {
    apply_direct(g_mode, g_voltage);
  }
}

} // namespace rollers