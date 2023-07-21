#include "../include/main.h"
#include "../globals/globals.hpp"
#include "../lib/movement.hpp"
#include "../lib/helper_functions.hpp"
#include "../lib/scoring.hpp"
#include "pros/vision.h"

 using namespace pros;

  void solo_awp() {
    // initial setup
    vector center = {};
    unsigned timeElapsed = 0;
    pros::Task track_time(stopwatch, &timeElapsed);
    setup_robot();
    pros::Task track_position(odometry, &center);
    move_straight(21.0, 50, &center);
    turn(40,-40,70, &center, true);
    move_straight(13.5, 50, &center);
    intake_flap.set_value(1); //raise intake
    //shoot triball
    pros::delay(400);
    catapult(400);
    //turn to face match load zone
    intake_flap.set_value(0); //raise intake
    turn(-40,40,235, &center, true);
    move_straight(41.0, 50, &center);
    intake_flap.set_value(1);//lower intake
    pros::delay(500);
    move_straight(-8.0,-50,&center);
    intake_flap.set_value(0); //raise intake
    turn(-40,40,135,&center);
    move_straight(20.0, 50, &center);
    turn(-40,40,90,&center);
    move_straight(18.0, 50, &center);

    /*
    moveToPoint(0,22,-22,&center);
    //turn(30,-30,75, &center);
    moveToPoint(27,18.5,-22,&center);
    intake_flap.set_value(1);
    pros::delay(300);
    cata_right=127;
    cata_left=127;
    pros::delay(400);
    cata_right=0;
    cata_left=0;
    intake_flap.set_value(0);
    //pickup match loads
    moveToPoint(-26,-32.5, -22,&center);
    
    intake_flap.set_value(1);
    pros::delay(500);
    move_straight(1.85, -40);
    intake_flap.set_value(0);
    turn(-40,40,133,&center,false);

    move_straight(2.8, 50);
    move(10,10);
    intake_flap.set_value(1);
    pros::delay(300);
    */
    //shoot
    //moveToPoint(-20,5,-22,&center);
    //intake
    //back up
    //raise intake
    //
   }