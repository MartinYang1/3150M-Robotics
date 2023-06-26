#include "../../include/main.h"
#include "globals.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);

// drive train
pros::Motor leftFrontMotor(10, MOTOR_GEAR_BLUE, 1);
pros::Motor rightFrontMotor(3, MOTOR_GEAR_BLUE);
pros::Motor leftBackMotor(8, MOTOR_GEAR_BLUE);
pros::Motor rightBackMotor(1, MOTOR_GEAR_BLUE, 1);
pros::Motor leftMidMotor(9, MOTOR_GEAR_BLUE, 1); 
pros::Motor rightMidMotor(2, MOTOR_GEAR_BLUE); 

// disc mechanisms

pros::Motor intake(90);
pros::Motor &roller = intake;
pros::Motor flywheel(90, MOTOR_GEAR_RED);  


// sensors
pros::Vision vision_sensor(90);
pros::Imu imu_sensor(4);
pros::Optical optical_sensor(90);

// pneumatics pistons

pros::ADIDigitalOut expander1_piston(90);
pros::ADIDigitalOut expander2_piston(90);
pros::ADIDigitalOut flywheel_indexer(90);


int autonSelection = SKILLS; // default auton selected