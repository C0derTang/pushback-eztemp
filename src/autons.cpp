#include "autons.hpp"
#include "distance_reset.hpp"
#include "rollers.hpp"
#include "main.h"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_forward_set(25.0, 0.0, 232.75);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_drive_constants_backward_set(23.3, 0.0, 283.75);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(9.6, 0.05, 55.5, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(7.4, 0.0, 87.5);           // Swing constants
  chassis.pid_odom_angular_constants_set(10, 0.0, 57.75);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(6.7, 0.0, 38.25);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(100_ms, 1_deg, 300_ms, 3_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(100_ms, 1_deg, 300_ms, 3_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

///
// sawping
///
void sob() {
  chassis.odom_xyt_set(12_in,-48_in, -90_deg);
  distanceReset(RESET_LEFT | RESET_BACK);
  
  chassis.pid_odom_set({{48_in,-48_in}, rev, DRIVE_SPEED});
  chassis.pid_wait();
  /*
  chassis.pid_turn_set(180, TURN_SPEED);
  chassis.pid_wait();
  tongue.set_value(1);
  pros::delay(500);
  
  distanceReset(RESET_LEFT);
  rollers::set_mode(rollers::Mode::Store);
  chassis.pid_odom_set({{48_in,-58_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  pros::delay(500);
  chassis.pid_odom_set({{48_in,-30_in}, rev, DRIVE_SPEED});
  chassis.pid_wait();
  rollers::set_mode(rollers::Mode::High);
*/
}

///
// 75 route
///
void skills() {
}

///
// 7ball hook wing left
///
void sevBallLeft() {

}

///
// 7ball hook wing right
///
void sevBallRight() {

}

///
// four ball mid 3 goal wing
///
void fourThreeLeft() {

}