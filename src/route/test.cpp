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
          pros::Task track_position(odometry2, &center);
     //move(50, 50);
     move_straight(48.0, 30, &center);
     pros::delay(500);
     move_straight(-48.0, -100, &center);

     track_position.remove();


 }
