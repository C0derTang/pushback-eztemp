// distance_reset.cpp
#include "distance_reset.hpp"

#include "subsystems.hpp"  // imu, leftWall, rightWall, backWall, chassis
#include <cmath>
#include <optional>

// ======================================
// Coordinate conventions (CENTER ORIGIN)
// ======================================
//
// GLOBAL / FIELD frame (inches):
//   (0,0) = CENTER of the field
//   +x = right,  -x = left
//   +y = forward, -y = backward
//
// Field size (typical VRC): 144" x 144"
//   Left wall:  x = -FIELD_W/2
//   Right wall: x = +FIELD_W/2
//   Back wall:  y = -FIELD_H/2
//   Front wall: y = +FIELD_H/2
//
// ROBOT frame (inches):
//   +x = right
//   +y = forward
//
// Heading theta (radians):
//   theta = 0   => robot +y aligns with global +y (forward)
//   theta = +90 => robot +y aligns with global +x (right)
//   theta is CCW-positive.
//
// Your IMU increases clockwise, so we negate it.
//
// ======================================

struct Pose2D {
  double x;      // inches
  double y;      // inches
  double theta;  // radians (global)
};

// ---- Field ----
static constexpr double FIELD_W_IN = 144.0;
static constexpr double FIELD_H_IN = 144.0;

static constexpr double WALL_LEFT_X  = -FIELD_W_IN / 2.0;
static constexpr double WALL_RIGHT_X = +FIELD_W_IN / 2.0;
static constexpr double WALL_BACK_Y  = -FIELD_H_IN / 2.0;
static constexpr double WALL_FRONT_Y = +FIELD_H_IN / 2.0;  // not used here

// ---- Sensor offsets from robot CENTER (robot frame), inches ----
// Per your specs:
static constexpr double LEFT_OFF_X_IN  = -3.5;
static constexpr double LEFT_OFF_Y_IN  = +5.75;

static constexpr double RIGHT_OFF_X_IN = +3.5;
static constexpr double RIGHT_OFF_Y_IN = +5.75;

static constexpr double BACK_OFF_X_IN  = +5.4;
static constexpr double BACK_OFF_Y_IN  = -1.5;

// ---- Sensor look directions (robot frame unit vectors) ----
static constexpr double LEFT_DIR_X  = -1.0;
static constexpr double LEFT_DIR_Y  =  0.0;

static constexpr double RIGHT_DIR_X = +1.0;
static constexpr double RIGHT_DIR_Y =  0.0;

static constexpr double BACK_DIR_X  =  0.0;
static constexpr double BACK_DIR_Y  = -1.0;

// PROS Distance returns millimeters
static inline double mm_to_in(double mm) { return mm / 25.4; }

// Valid distance range filter (tune)
static constexpr double MIN_VALID_IN = 1.0;
static constexpr double MAX_VALID_IN = 120.0;

static inline double deg_to_rad(double deg) { return deg * M_PI / 180.0; }
static inline double rad_to_deg(double rad) { return rad * 180.0 / M_PI; }

// IMU increases clockwise; math expects CCW-positive.
// If you need an offset, do: return deg_to_rad(-(imuDeg - offsetDeg));
static inline double headingDegToThetaRad(double imuDeg) {
  return deg_to_rad(-imuDeg);
}

// Rotate robot-frame vector (x right, y forward) into global frame,
// where theta is CCW from global +y (forward).
static inline void rotRobotToGlobal(double rx, double ry, double theta, double &gx, double &gy) {
  const double c = std::cos(theta);
  const double s = std::sin(theta);

  // theta=0 => (rx,ry) stays (rx,ry)
  // theta=+90deg => robot forward (+y) becomes global +x
  gx = rx * c + ry * s;
  gy = -rx * s + ry * c;
}

static inline std::optional<double> readDistInches(pros::Distance &d) {
  const double in = mm_to_in(static_cast<double>(d.get()));
  if (in < MIN_VALID_IN || in > MAX_VALID_IN) return std::nullopt;
  return in;
}

// Solve X from LEFT wall using left sensor ray
static inline std::optional<double> solveRobotXFromLeft(
    double wallLeftX, double dL,
    double left_off_gx, double left_dir_gx) {
  if (std::fabs(left_dir_gx) < 1e-6) return std::nullopt;
  return wallLeftX - dL * left_dir_gx - left_off_gx;
}

// Solve X from RIGHT wall using right sensor ray
static inline std::optional<double> solveRobotXFromRight(
    double wallRightX, double dR,
    double right_off_gx, double right_dir_gx) {
  if (std::fabs(right_dir_gx) < 1e-6) return std::nullopt;
  return wallRightX - dR * right_dir_gx - right_off_gx;
}

// Solve Y from BACK wall using back sensor ray
static inline std::optional<double> solveRobotYFromBack(
    double wallBackY, double dB,
    double back_off_gy, double back_dir_gy) {
  if (std::fabs(back_dir_gy) < 1e-6) return std::nullopt;
  return wallBackY - dB * back_dir_gy - back_off_gy;
}

// ---------- IMPORTANT ----------
// Partial reset requires "current pose" getters and a pose setter.
// Different EZ-Template versions name these differently.
// Below we provide a small adapter layer.
//
// ✅ Edit these 3 functions to match your EZ-Template API if needed.
// -------------------------------

// Try to read current odom pose. Return nullopt if you don't have getters.
static std::optional<Pose2D> getCurrentPose() {
  // Examples you might have (uncomment what exists in your project):
  //
  auto p = chassis.odom_pose_get() ;
  return Pose2D{p.x, p.y, deg_to_rad(p.theta)};
}

// Apply pose to odom. Return false if you haven't wired it yet.
static bool setPoseToOdom(const Pose2D &p) {
  // Uncomment ONE that compiles:
  //
  // chassis.set_pose(p.x, p.y, rad_to_deg(p.theta));
  chassis.odom_xyt_set(p.x, p.y, rad_to_deg(p.theta));
  // chassis.odom_set_pose({p.x, p.y, rad_to_deg(p.theta)});
  //
  return true;
}

// If you cannot read current pose, choose what to do when only one axis solves.
// Option A: don't apply anything unless both axes solved.
// Option B: apply solved axis and set the other axis to 0 (NOT recommended).
static constexpr bool REQUIRE_GETTERS_FOR_PARTIAL = true;

// Compute pose components from IMU + selected sensors.
// Returns (maybeX, maybeY, theta). Theta is always computed.
struct PartialSolve {
  std::optional<double> x;
  std::optional<double> y;
  double theta;
};

static std::optional<PartialSolve> solvePartial(uint8_t mask) {
  const bool wantLeft  = (mask & RESET_LEFT)  != 0;
  const bool wantRight = (mask & RESET_RIGHT) != 0;
  const bool wantBack  = (mask & RESET_BACK)  != 0;

  // Must request at least one sensor
  if (!wantLeft && !wantRight && !wantBack) return std::nullopt;

  const double theta = headingDegToThetaRad(imu.get_heading());

  // Read only requested sensors
  std::optional<double> dL, dR, dB;
  if (wantLeft)  dL = readDistInches(leftWall);
  if (wantRight) dR = readDistInches(rightWall);
  if (wantBack)  dB = readDistInches(backWall);

  // Rotate offsets + directions
  double left_off_gx, left_off_gy, left_dir_gx, left_dir_gy;
  rotRobotToGlobal(LEFT_OFF_X_IN, LEFT_OFF_Y_IN, theta, left_off_gx, left_off_gy);
  rotRobotToGlobal(LEFT_DIR_X, LEFT_DIR_Y, theta, left_dir_gx, left_dir_gy);

  double right_off_gx, right_off_gy, right_dir_gx, right_dir_gy;
  rotRobotToGlobal(RIGHT_OFF_X_IN, RIGHT_OFF_Y_IN, theta, right_off_gx, right_off_gy);
  rotRobotToGlobal(RIGHT_DIR_X, RIGHT_DIR_Y, theta, right_dir_gx, right_dir_gy);

  double back_off_gx, back_off_gy, back_dir_gx, back_dir_gy;
  rotRobotToGlobal(BACK_OFF_X_IN, BACK_OFF_Y_IN, theta, back_off_gx, back_off_gy);
  rotRobotToGlobal(BACK_DIR_X, BACK_DIR_Y, theta, back_dir_gx, back_dir_gy);

  PartialSolve out;
  out.theta = theta;

  // Y from back wall if requested & valid
  if (wantBack && dB) {
    out.y = solveRobotYFromBack(WALL_BACK_Y, *dB, back_off_gy, back_dir_gy);
  }

  // X from left/right if requested & valid
  // Preference: LEFT first if both selected and both valid
  if (wantLeft && dL) {
    out.x = solveRobotXFromLeft(WALL_LEFT_X, *dL, left_off_gx, left_dir_gx);
  }
  if (!out.x && wantRight && dR) {
    out.x = solveRobotXFromRight(WALL_RIGHT_X, *dR, right_off_gx, right_dir_gx);
  }

  return out;
}

bool distanceReset(uint8_t mask) {
  auto partialOpt = solvePartial(mask);
  if (!partialOpt) return false;

  const auto partial = *partialOpt;

  // Need at least one solved axis to do anything useful
  if (!partial.x && !partial.y) return false;

  // If partial reset is allowed, we keep the other axis from current odom pose.
  Pose2D target{0.0, 0.0, partial.theta};

  if (partial.x && partial.y) {
    // Full reset
    target.x = *partial.x;
    target.y = *partial.y;
  } else {
    // Partial reset
    auto curOpt = getCurrentPose();
    if (!curOpt) {
      if (REQUIRE_GETTERS_FOR_PARTIAL) return false;
      // If you insist on applying anyway, you could set missing axis to 0 here.
      target.x = partial.x.value_or(0.0);
      target.y = partial.y.value_or(0.0);
    } else {
      const Pose2D cur = *curOpt;
      target.x = partial.x.value_or(cur.x);
      target.y = partial.y.value_or(cur.y);
      // Heading: choose whether you want to keep current heading or override with IMU heading.
      // Here we override with IMU-derived heading (more consistent for wall resets).
      target.theta = partial.theta;
    }
  }

  return setPoseToOdom(target);
}