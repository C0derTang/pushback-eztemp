#include "distance_reset.hpp"

#include <cmath>
#include <algorithm>

#include "subsystems.hpp"
#include "EZ-Template/api.hpp"

static constexpr double LEFT_OFF_X_IN  = -3.5;
static constexpr double LEFT_OFF_Y_IN  = +5.75;

static constexpr double RIGHT_OFF_X_IN = +3.5;
static constexpr double RIGHT_OFF_Y_IN = +5.75;

static constexpr double BACK_OFF_X_IN  = +5.4;
static constexpr double BACK_OFF_Y_IN  = -1.5;

static constexpr double WALL_LEFT_X  = -72.0;
static constexpr double WALL_RIGHT_X = +72.0;
static constexpr double WALL_BACK_Y  = -72.0;
static constexpr double WALL_FRONT_Y = +72.0;

static inline double mm_to_in(double mm) { return mm / 25.4; }

static inline double wrap_deg_360(double deg) {
  deg = std::fmod(deg, 360.0);
  if (deg < 0) deg += 360.0;
  return deg;
}

struct Vec2 { double x, y; };

static inline Vec2 forward_unit(double heading_deg) {
  double h = heading_deg * M_PI / 180.0;
  return { std::sin(h), std::cos(h) };
}

static inline Vec2 right_unit(double heading_deg) {
  double h = heading_deg * M_PI / 180.0;
  return { std::cos(h), -std::sin(h) };
}

static inline Vec2 offset_robot_to_global(double heading_deg, double ox_in, double oy_in) {
  const Vec2 f = forward_unit(heading_deg);
  const Vec2 r = right_unit(heading_deg);
  return { ox_in * r.x + oy_in * f.x, ox_in * r.y + oy_in * f.y };
}

static inline bool valid_mm(double mm, double min_mm, double max_mm) {
  return (mm >= min_mm && mm <= max_mm);
}

static constexpr double EPS = 1e-6;

static void apply_sensor_ray(
    double heading_deg,
    double d_in,
    double off_x_in, double off_y_in,
    Vec2 u,                // beam direction in global coords (unit-ish)
    bool &have_x, bool &have_y,
    double &x_sum, int &x_n,
    double &y_sum, int &y_n
) {
  const Vec2 off = offset_robot_to_global(heading_deg, off_x_in, off_y_in);

  // If the ray points more along X, it hits a vertical wall (x = const)
  if (std::fabs(u.x) > std::fabs(u.y) + EPS) {
    const double wall_x = (u.x > 0) ? WALL_RIGHT_X : WALL_LEFT_X;

    // wall_x = (cx + off.x) + u.x * d  => cx = wall_x - off.x - u.x*d
    const double cx = wall_x - off.x - (u.x * d_in);
    x_sum += cx; x_n++; have_x = true;
  } else if (std::fabs(u.y) > EPS) {
    // Otherwise it hits a horizontal wall (y = const)
    const double wall_y = (u.y > 0) ? WALL_FRONT_Y : WALL_BACK_Y;

    // wall_y = (cy + off.y) + u.y * d  => cy = wall_y - off.y - u.y*d
    const double cy = wall_y - off.y - (u.y * d_in);
    y_sum += cy; y_n++; have_y = true;
  }
}

void distanceReset(uint8_t flags, double min_mm, double max_mm) {
  const double cur_x_in = chassis.odom_x_get();
  const double cur_y_in = chassis.odom_y_get();
  const double heading_deg = wrap_deg_360(chassis.odom_theta_get());

  const Vec2 f = forward_unit(heading_deg);
  const Vec2 r = right_unit(heading_deg);

  bool have_x = false, have_y = false;
  double x_sum = 0.0, y_sum = 0.0;
  int x_n = 0, y_n = 0;

  // Left sensor beam points robot-left = -r
  if (flags & RESET_LEFT) {
    const double mm = static_cast<double>(leftWall.get());
    if (valid_mm(mm, min_mm, max_mm)) {
      const double d_in = mm_to_in(mm);
      apply_sensor_ray(heading_deg, d_in, LEFT_OFF_X_IN, LEFT_OFF_Y_IN,
                       Vec2{-r.x, -r.y},
                       have_x, have_y, x_sum, x_n, y_sum, y_n);
    }
  }

  // Right sensor beam points robot-right = +r
  if (flags & RESET_RIGHT) {
    const double mm = static_cast<double>(rightWall.get());
    if (valid_mm(mm, min_mm, max_mm)) {
      const double d_in = mm_to_in(mm);
      apply_sensor_ray(heading_deg, d_in, RIGHT_OFF_X_IN, RIGHT_OFF_Y_IN,
                       Vec2{r.x, r.y},
                       have_x, have_y, x_sum, x_n, y_sum, y_n);
    }
  }

  // Back sensor beam points robot-back = -f
  if (flags & RESET_BACK) {
    const double mm = static_cast<double>(backWall.get());
    if (valid_mm(mm, min_mm, max_mm)) {
      const double d_in = mm_to_in(mm);
      apply_sensor_ray(heading_deg, d_in, BACK_OFF_X_IN, BACK_OFF_Y_IN,
                       Vec2{-f.x, -f.y},
                       have_x, have_y, x_sum, x_n, y_sum, y_n);
    }
  }

  double new_x_in = cur_x_in;
  double new_y_in = cur_y_in;

  if (have_x && x_n > 0) new_x_in = x_sum / x_n;
  if (have_y && y_n > 0) new_y_in = y_sum / y_n;

  chassis.odom_xyt_set(new_x_in * 1_in, new_y_in * 1_in, heading_deg * 1_deg);
}