#include "../../include/main.h"
#include "globals.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);

// drive train
pros::Motor leftFrontMotor(18, MOTOR_GEAR_BLUE, 1);
pros::Motor rightFrontMotor(8, MOTOR_GEAR_BLUE);
pros::Motor leftBackMotor(20, MOTOR_GEAR_BLUE);
pros::Motor rightBackMotor(10, MOTOR_GEAR_BLUE, 1);
pros::Motor leftMidMotor(19, MOTOR_GEAR_BLUE, 1); 
pros::Motor rightMidMotor(9, MOTOR_GEAR_BLUE); 

//cata


// other mechanisms 
pros::Motor cata_right(5);//cata right
pros::Motor cata_left(1,1); //cata left
pros::Motor intake(90); //cata left


pros::Motor &roller = intake;
pros::Motor flywheel(2, MOTOR_GEAR_RED,1);  
pros::ADIDigitalOut expander1_piston(90);
pros::ADIDigitalOut expander2_piston(5);

// sensors
pros::Vision vision_sensor(90);
pros::Imu imu_sensor(21);
pros::Optical optical_sensor(90);
pros::ADIDigitalOut flywheel_indexer(90);
pros::ADIDigitalOut intake_flap(7);

// pneumatics pistons

pros::ADIDigitalOut lift_release(5);
pros::ADIDigitalOut puncher_release(6);
pros::ADIDigitalOut side_panel(8);





int autonSelection = SKILLS; // default auton selected