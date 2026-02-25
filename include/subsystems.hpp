#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "pros/distance.hpp"
#include "pros/imu.hpp"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');
inline pros::Motor intake(-3, pros::MotorGearset::blue);
inline pros::Motor indexer(2, pros::MotorGearset::green);
inline pros::Motor roller(1, pros::MotorGearset::green);

inline pros::adi::DigitalOut hood('e');
inline pros::adi::DigitalOut tongue('h');
inline pros::adi::DigitalOut mid('f');
inline pros::adi::DigitalOut wing('g');

inline pros::Distance backWall(21);
inline pros::Distance leftWall(4);
inline pros::Distance rightWall(9);