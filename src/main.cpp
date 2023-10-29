#include "main.h"
#include "selection.h"
#include "PotMgr/PotMgr.hpp"
#include "globals.hpp"
#include <iostream>
#include <string>
#define stringify( name ) #name

/*-----------------------------------------------------------------------------
  __  __         _
 |  \/  |  __ _ (_) _ __
 | |\/| | / _` || || '_ \
 | |  | || (_| || || | | |
 |_|  |_| \__,_||_||_| |_|

Created on 5/15/2022
Last Updated on 3/20/2023

The Main function initializes and runs our code.
It sets up autonomous, driver control, and everything competiton.
Main is the most important program because it expands to all others.

-----------------------------------------------------------------------------*/

PotMgr autoSwitches1('H'); // Initialize Autonomous Selection Potentiometer
pros::ADIAnalogIn clogf_sensor ('E');

// ---------------- TASK ---------------- //
// Iterates all functions that are designed to run in the background
// Does not include the task of EZ-Template, to prevent unforseen merging issues
void taskFunctions(){
	int time_elapsed=0; // Declare the time in usable values

	pros::delay(3000);

	while(true){ // Loop Infinitely

		if(time_elapsed%300==0){ // For every 300ms

			// The following code will be toggleable at the competition
			if(ctrlerDebugOn){ // When Controller Debug is enabled

				// Print the currently-selected autonomous program to the controller
				int automas = std::floor(autoSwitches1.get_value()); // Get index of current autonomous option
				//autoTitles currentMode = static_cast<autoTitles>(automas); // Store autonomous option with 'AutoTitles' data type
				printRow(1,stringify(currentMode)); // Print out the name of the autonomous 

				
			}

			brainPosAyncIterate();

		}

		liftAsyncIterate();

		setShooterAsyncIterate(); // Flywheel Feed-Forward and PID 

		detect_motor_disconnect();

		// Wait 20ms for sensor values to update
		time_elapsed+=20;
		pros::delay(20);
	}
}

pros::Task runTasks(taskFunctions); // Instantiate Task

// ---------------- INITIALIZE ---------------- //
// Sets up all sensors, motors and tasks
void initialize() {
	runTasks.suspend(); // Turn off task for now
	shooterMtr.set_voltage(0); // Reset Flywheel voltage output

	initializeGeneral(); // Callibrate Sensors
	selectorInit();
}


// ---------------- DISABLED ---------------- //
// Period of program when robot is disabled
void disabled() {
	runTasks.suspend(); // Turn off task for now
}


// ---------------- COMPETITION INITIALIZE ---------------- //
// Initialize sensors and others at the start of a comp
// Don't start autonous until the encoders are callibrated
void competition_initialize() {}

// ---------------- AUTONOMOUS ---------------- //
// The robot drives on its own
// Starts autnomous from a chosen autonomous
void autonomous() {

	runTasks.resume(); // Enable useful tasks

	// Run initialization code for autonomous
	initializeAutonomous();

	// Find the currently selected auto from the potentiomter selector.
  	int automas = std::floor(autoSwitches1.get_value());

	switch (autonSelection)
	{
		case S_RED5: //autoroute 2
			Awp();
			break;
		case S_BLUE5:
			Awp();
			break;
		case S_RED7: S_BLUE7:
			Awp(); break;
		case S_RED8: S_BLUE8:
			Awp(); break;
		case B_RED3: B_BLUE3:
			Awp(); break;
		case B_RED6_C: B_BLUE6_C:
			Awp(); break;

	}
  	// Check through each possible autonomous to find the correct one
//   	switch (automas) {

//     // ------ SKILLS AUTONOMOUS ----- //
//     case 1:
// 		chassis.enable_odometry();
// 		// chassis.set_exit_condition(chassis.turn_exit,  0, 3,  500, 7,   500, 500);
// 		// chassis.set_exit_condition(chassis.drive_exit, 20,  50, 300, 150, 500, 500);
//     	Skills();
//       	break;

//     // ----- SKILLS AUTONOMOUS VARIENT ----- //
//     case 2:
//       	Skills();
//       	break;

//     // ----- WIN POINT ----- //
//     case 3:
// 		chassis.set_exit_condition(chassis.turn_exit,  30, 3,  500, 7,   500, 500);
//       	Awp();
//       	break;

//     // ----- WIN POINT VARIENT ----- //
//     case 4:
// 	// chassis.set_exit_condition(chassis.turn_exit,  0, 3,  500, 7,   500, 500);
// 	chassis.set_exit_condition(chassis.drive_exit, 20,  50, 300, 150, 500, 500);
// 	chassis.enable_odometry();

//       	Awp_Auto_Stack();
//       	break;

//     // ----- LEFT SIDE SUPPORT ----- //
//     case 5:
//       	Left_Side();
//       	break;

//     // ----- LEFT SIDE SUPPORT VARIENT ----- //
//     case 6:
//       	Left_Side_Auto_Stack();
//       	break;

//     // ----- RIGHT SIDE SUPPORT ----- //
//     case 7:
// 	chassis.set_exit_condition(chassis.turn_exit,  30, 3,  500, 7,   500, 500);

//       	Right_Side();
//       	break;

//     // ----- RIGHT SIDE SUPPORT + AUTO STACK ----- //
//     case 8:
//       	Right_Side_Auto_Stack();
//       	break;
//   }
}

void move(const int leftVolt, const int rightVolt){
    leftFrontMotor = leftVolt; leftMidMotor = leftVolt; leftBackMotor = leftVolt;
    rightFrontMotor = rightVolt; rightMidMotor = rightVolt; rightBackMotor = rightVolt;
}

// ---------------- DRIVER CONTROL ---------------- //
// Robot is driven by a human
// Run the robot based on joystick and button inputs
// Display Odometry details
void opcontrol() {
    expander1_piston.set_value(0);
    expander2_piston.set_value(0);

    flywheel_indexer.set_value(0);
    int intake_state=1;
    int flywheel_state=0;
    bool shooterReady = false;
    while (true) {
        int power = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turnRate = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        move(power + turnRate, power - turnRate);
        if(master.get_digital(DIGITAL_R1))
        {
            cata_right=127;
            cata_left=127; // cata motors
            move(MOTOR_BRAKE_HOLD, MOTOR_BRAKE_HOLD);

        }
        else
        {
            cata_right=MOTOR_BRAKE_HOLD;
            cata_left=MOTOR_BRAKE_HOLD;
        }
        if (master.get_digital(DIGITAL_Y) )
        {
            expander2_piston.set_value(1);
        }
        else if (master.get_digital(DIGITAL_A))
        {
            expander2_piston.set_value(0);
        }
        if (master.get_digital(DIGITAL_L1) )
        {
            intake_flap.set_value(0);
        }
        else if (master.get_digital(DIGITAL_L2) )
        {
            intake_flap.set_value(1);
        }

        if (master.get_digital(DIGITAL_UP) && master.get_digital(DIGITAL_DOWN) &&master.get_digital(DIGITAL_RIGHT) &&master.get_digital(DIGITAL_LEFT))

        {
            //deploy pistons
        }
        else
        {
            //don't deploy pistons
        }
        
    }
}
