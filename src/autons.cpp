#include "autons.hpp"
#include "distance_reset.hpp"
#include "globals.hpp"
#include "pros/device.hpp"
#include "pros/rtos.hpp"
#include "rollers.hpp"
#include "main.h"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 127;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_forward_set(21.2, 0.0, 90);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_drive_constants_backward_set(20.2, 0.0, 110);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(0.0, 0.0, 0.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(9.6, 0.05, 55.5, 15.0);  
  chassis.pid_swing_constants_set(0, 0.0, 0);           // Swing constants
  chassis.pid_odom_angular_constants_set(14.2, 0.0, 100);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(0, 0.0, 0);  // Angular control for boomerang motions

  // Exit conditions

  chassis.pid_turn_exit_condition_set(100_ms, 3_deg, 50000_ms, 7_deg, 500_ms, 500_ms);
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
  auton_running = true;
  rollers::set_voltage(12000);
  chassis.odom_xyt_set(0_in,-48_in, 270_deg);
  pros::delay(10);
  distanceReset(RESET_LEFT | RESET_BACK);
  
  chassis.pid_odom_set({{49_in,-48_in}, rev, 80});
  chassis.pid_wait();
  
  chassis.pid_turn_set(180, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  
  tongue.set_value(1);
  pros::delay(200);

  distanceReset(RESET_LEFT);
  rollers::set_mode(rollers::Mode::Store);
  chassis.pid_odom_set({{48_in,-59.867_in}, fwd, 70});
  pros::delay(1200);
  chassis.pid_odom_set({{49_in,-27_in}, rev, 100});
  pros::delay(800);
  rollers::set_mode(rollers::Mode::High);
  pros::delay(1000);
  tongue.set_value(0);
  chassis.pid_turn_set(270, TURN_SPEED);
  chassis.pid_wait_quick();
  //distanceReset(RESET_LEFT);

  chassis.pid_odom_set({{12_in,-28_in}, fwd, 100});
  rollers::set_mode(rollers::Mode::Store);
  pros::delay(600);
  tongue.set_value(1);
  pros::delay(270);
  tongue.set_value(0);
  chassis.pid_turn_set(270, TURN_SPEED);
  chassis.pid_wait_quick();
  //distanceReset(RESET_LEFT);
  chassis.pid_drive_set(20_in, 70);
  pros::delay(300);
  tongue.set_value(1);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(225_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_odom_set({{-36_in,-46_in}, fwd, 90});
  chassis.pid_wait_quick();
  chassis.pid_turn_set(180, TURN_SPEED);
  chassis.pid_wait();
  distanceReset(RESET_RIGHT);
  chassis.pid_odom_set({{-49_in,-30_in}, rev, 100});
  pros::delay(200);
  rollers::set_mode(rollers::Mode::High);
  pros::delay(1200);
  chassis.odom_y_set(-30_in);

  chassis.pid_odom_set({{-49.5_in,-59_in}, fwd, 76});
  rollers::set_mode(rollers::Mode::Store);
  pros::delay(1450);
  distanceReset(RESET_RIGHT);
  chassis.pid_odom_set({{-48_in,-48_in}, rev, 100});
  chassis.pid_wait_quick();
  distanceReset(RESET_RIGHT);

  chassis.pid_turn_set(225_deg, TURN_SPEED);
  chassis.pid_wait_quick();
  tongue.set_value(0);
  chassis.pid_odom_set({{-12_in,-12.67_in}, rev, 100});
  pros::delay(1000);
  rollers::set_mode(rollers::Mode::Mid);
  chassis.pid_wait_quick();
  pros::delay(2000);
  auton_running = false;
}

///
// 75 route
///
void skills() {
    auton_running = true;
  rollers::set_voltage(12000);
  chassis.odom_xyt_set(0_in,-48_in, 90_deg);
    chassis.odom_xyt_set(-18_in,-48_in, 90_deg);
  pros::delay(10);
  distanceReset(RESET_RIGHT | RESET_BACK);
  
  chassis.pid_odom_set({{-49_in,-48_in}, rev, 80});
  chassis.pid_wait();
  
  chassis.pid_turn_set(180, 100);
  chassis.pid_wait_quick_chain();
  
  tongue.set_value(1);
  pros::delay(200);

  distanceReset(RESET_RIGHT);
  rollers::set_mode(rollers::Mode::Store);
  chassis.pid_odom_set({{-47.3_in,-60_in}, fwd, 80});
  pros::delay(2500);
  chassis.pid_odom_set({{-49_in,-48_in}, rev, 80});
  chassis.pid_wait();
  chassis.pid_odom_set({{-56_in,-36_in}, rev, 100});
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(180, 90);
  chassis.pid_wait();
  distanceReset(RESET_RIGHT | RESET_BACK);
  chassis.pid_odom_set({{-60_in,40_in}, rev, 100});
  chassis.pid_wait_quick();
  distanceReset(RESET_RIGHT | RESET_BACK);
  pros::delay(10);
  chassis.pid_turn_set(90, 90);
  chassis.pid_wait();
  distanceReset(RESET_LEFT | RESET_BACK);
  pros::delay(10);

  chassis.pid_odom_set({{-52_in,43_in}, fwd, 100});
  chassis.pid_wait_quick();
  chassis.pid_turn_set(0, 90);
  chassis.pid_wait();
  distanceReset(RESET_LEFT);

  chassis.pid_odom_set({{-50_in,28_in}, rev, 100});
  pros::delay(670);
  rollers::set_mode(rollers::Mode::High);
  pros::delay(2000);
  distanceReset(RESET_LEFT);
  chassis.pid_turn_set(0, 90);
  chassis.pid_wait();
  chassis.pid_drive_set(26.7_in, 76);
  pros::delay(500);
  rollers::set_mode(rollers::Mode::Store);
  pros::delay(2500);
  chassis.pid_odom_set({{-50_in,28_in}, rev, 100});
  pros::delay(670);
  rollers::set_mode(rollers::Mode::High);
  pros::delay(2000);
  tongue.set_value(0);
  chassis.pid_drive_set(13_in, 80);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(90, 90);
  chassis.pid_wait();
  chassis.pid_drive_set(13_in, 80);
  chassis.pid_wait_quick();
  distanceReset(RESET_LEFT | RESET_BACK);

}

///
// 7ball hook wing left
///
void sevBallLeft() {
  auton_running = true;
  chassis.pid_odom_set({{0_in,48_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  auton_running = false;

}

///
// 7ball hook wing right
///
void sevBallRight() {
  auton_running = true;
  rollers::set_voltage(12000);
  chassis.odom_xyt_set(18_in,-56_in, 0_deg);
  pros::delay(10);
  distanceReset(RESET_RIGHT | RESET_BACK);
  rollers::set_mode(rollers::Mode::Store);
  chassis.pid_odom_set({{23_in,-27_in}, fwd, 100});
  pros::delay(467);
  tongue.set_value(true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(320, 100);
  chassis.pid_wait_quick();
  chassis.pid_odom_set({{43_in,-50_in}, rev, 120});
  chassis.pid_wait_quick();
  chassis.pid_turn_set(180, 100);
  pros::delay(967);
  chassis.drive_set(0, 0);
  distanceReset(RESET_LEFT);
  
  chassis.pid_odom_set({{47.5_in,-64.5_in}, fwd, 80});
  pros::delay(1267);
  
  chassis.pid_odom_set({{50_in,-28_in}, rev, 90});
  pros::delay(1000);
  rollers::set_mode(rollers::Mode::High);
  tongue.set_value(0);
  pros::delay(1867);
  wing.set_value(0);
  chassis.odom_xy_set(0_in,0_in);
  chassis.pid_odom_set({{6_in,-7_in}, fwd, 100});
  chassis.pid_wait_quick(); 
  chassis.pid_odom_set({{5.5_in,12_in}, rev, 100});
  rollers::set_mode(rollers::Mode::Stop);

  chassis.pid_wait();
}

///
// four ball mid 3 goal wing
///
void fourThreeLeft() {
  auton_running = true;
  rollers::set_voltage(12000);
  chassis.odom_xyt_set(-18_in,-48_in, 90_deg);
  pros::delay(10);
  distanceReset(RESET_RIGHT | RESET_BACK);
  
  chassis.pid_odom_set({{-49_in,-48_in}, rev, 80});
  chassis.pid_wait();
  
  chassis.pid_turn_set(180, 100);
  chassis.pid_wait_quick();
  
  tongue.set_value(1);
  pros::delay(200);

  distanceReset(RESET_RIGHT);
  rollers::set_mode(rollers::Mode::Store);
  chassis.pid_odom_set({{-47.67_in,-60.5_in}, fwd, 70});
  pros::delay(1200);
  chassis.pid_odom_set({{-48.5_in,-28_in}, rev, 100});
  pros::delay(800);
  rollers::set_mode(rollers::Mode::High);
  tongue.set_value(0);
  pros::delay(1300);
  chassis.pid_drive_set(6_in, 60);
  chassis.pid_wait_quick();
  rollers::set_mode(rollers::Mode::Store);
  chassis.pid_drive_set(-10_in, 120);
  pros::delay(800);

  chassis.pid_turn_set(90, 127);
  chassis.pid_wait();
  chassis.drive_set(0, 0);
  pros::delay(100);
  distanceReset(RESET_RIGHT | RESET_BACK);
  pros::delay(100);
  chassis.pid_odom_set({{-24_in,-25_in}, fwd, 100});
  rollers::set_mode(rollers::Mode::Store);

  pros::delay(200);
  tongue.set_value(1);
  chassis.pid_wait_quick();
  distanceReset(RESET_RIGHT | RESET_BACK);
  pros::delay(100);
  chassis.pid_odom_set({{-25_in,-25_in}, rev, 100});
  pros::delay(670);
  chassis.pid_turn_set(222, 127);
  rollers::set_mode(rollers::Mode::Low);
  pros::delay(100);
  rollers::set_mode(rollers::Mode::Store);

  chassis.pid_wait();
  chassis.pid_drive_set(-16_in, 80);

  //chassis.pid_odom_set({{-16_in,-12_in, 225_deg}, rev, 70});
  
  pros::delay(400);
  rollers::set_mode(rollers::Mode::Mid);
  chassis.pid_wait();
  pros::delay(967);

  //chassis.odom_xy_set(-10_in,-10_in);
  
  
  chassis.pid_odom_set({{-35_in,-30_in}, fwd, 90});
  tongue.set_value(0);

  rollers::set_mode(rollers::Mode::Stop);
  
  chassis.pid_wait_quick();
  chassis.pid_turn_set(180, TURN_SPEED);
  chassis.pid_wait_quick();
  distanceReset(RESET_RIGHT);
  wing.set_value(0);
  chassis.pid_odom_set({{-38_in,-18.67_in}, rev, 100});
  chassis.pid_wait_quick();
  pros::delay(3000);
}