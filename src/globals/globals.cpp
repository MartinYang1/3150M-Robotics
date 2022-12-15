#include "../../include/main.h"
#include "globals.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);

// drive train
pros::Motor leftFrontMotor(1, MOTOR_GEAR_BLUE, 1);
pros::Motor rightFrontMotor(2, MOTOR_GEAR_BLUE);
pros::Motor leftBackMotor(3, MOTOR_GEAR_BLUE, 1);
pros::Motor rightBackMotor(4, MOTOR_GEAR_BLUE);
pros::Motor leftMidMotor(5, MOTOR_GEAR_BLUE, 1); 
pros::Motor rightMidMotor(6, MOTOR_GEAR_BLUE); 

// disc mechanisms
pros::Motor intake(10);
pros::Motor &roller = intake;
pros::Motor flywheel(21, MOTOR_GEAR_RED);  

// sensors
pros::Vision vision_sensor(19);
pros::Imu imu_sensor(11);
pros::Optical optical_sensor(20);

// pneumatics pistons
pros::ADIDigitalOut expander1_piston(8);
pros::ADIDigitalOut expander2_piston(3);
pros::ADIDigitalOut flywheel_indexer(1);

int autonSelection = SKILLS; // default auton selected