#include "../include/main.h"
#include "../globals/globals.hpp"
#include "../lib/movement.hpp"
#include "../lib/helper_functions.hpp"
#include "../lib/scoring.hpp"
#include "pros/vision.h"

using namespace pros;

 void test() {
//     // initial setup
     setup_robot();
     vector center = {};
     pros::Task track_position(odometry, &center);
     //move(50, 50);
     // move_straight(-48.0, &center);     
     // pros::delay(2000);
     // move_straight(48.0, &center);

     moveToPoint(-30, -30, 0, &center);
     pros::delay(200);
     moveToPoint(0, 0, 0, &center);
     //   turn(-20, 20, 270, &center, false);
     // std::cout << "Hi";
     // turn(20, -20, 90, &center);
     // track_position.remove();


 }
