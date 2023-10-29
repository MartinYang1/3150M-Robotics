#ifndef GLOBALS_H
#define GLOBALS_H

#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.hpp"

extern pros::Controller master;

// drive train
extern pros::Motor leftFrontMotor;
extern pros::Motor rightFrontMotor;
extern pros::Motor leftBackMotor;
extern pros::Motor rightBackMotor;
extern pros::Motor leftMidMotor;
extern pros::Motor rightMidMotor;

// disc mechanisms
extern pros::Motor cata_right;
extern pros::Motor cata_left;
extern pros::Motor intake;
extern pros::Motor flywheel;

extern pros::Motor &roller;
extern pros::Motor &indexer;

// sensors
extern pros::Vision vision_sensor;
extern pros::Imu imu_sensor;
extern pros::Optical optical_sensor;

// pneumatics
extern pros::ADIDigitalOut expander1_piston;
extern pros::ADIDigitalOut expander2_piston;
extern pros::ADIDigitalOut flywheel_indexer;
extern pros::ADIDigitalOut intake_flap;


#endif