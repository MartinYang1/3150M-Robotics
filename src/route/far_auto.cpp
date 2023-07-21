#include "../include/main.h"
#include "../globals/globals.hpp"
#include "../lib/movement.hpp"
#include "../lib/helper_functions.hpp"
#include "../lib/scoring.hpp"
#include "pros/vision.h"

using namespace pros;

 void far_auto() { 
        vector center = {};

        unsigned timeElapsed = 0;
        setup_robot();
        pros::Task track_position(odometry, &center);
        //route starts here
        //lower intake
        //approach the goal
        intake_flap.set_value(1); //intake down
        //turn toward goal
        pros::delay(400);
        move_straight(25.0, 50, &center);
        turn(-40,40,325,&center);
        move_straight(10.0,35,&center);
        turn(35,-35,90,&center);
        intake_flap.set_value(0); //intake up
        move_straight(0.5,110);//score into goal
        move_straight(-15.0,-50,&center);
        turn(-40,40,359,&center);
        move_straight(5.0,50,&center);
        intake_flap.set_value(1); //intake down

       

    //
    track_position.remove();
    master.print(0, 0, "%d", timeElapsed);

 }