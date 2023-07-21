#include "../include/main.h"
#include "globals/globals.hpp"
#include "pros/misc.h"
#include "lib/movement.hpp"
#include "lib/scoring.hpp"
#include "lib/helper_functions.hpp"

void opcontrol() {
         vector center = {};
          pros::Task track_position(odometry, &center);

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
    track_position.remove();
}