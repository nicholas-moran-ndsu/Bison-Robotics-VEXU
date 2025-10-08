#include "autons/autons.hpp"
#include "pros/apix.h"

using namespace pros; 

void autons::run(AutonRoutine which) {
  switch (which) {
    case AutonRoutine::NearSide:
      // Example: intake while driving forward, settle, turn, strafe
      intake_set(127);                         // run intake in parallel
      drive_in(24, 120);                       // 24" forward
      wait_ms(150);                            // let game pieces settle
      turn_by_deg(+90, 90);                    // IMU turn +90°
      strafe_right_in(8, 100);                 // 8" right
      intake_set(0);
      break;

    case AutonRoutine::FarSide:
      drive_in(12, 120);
      turn_to_deg(0, 100);                     // face field "forward"
      strafe_right_in(12, 100);
      break;

    case AutonRoutine::Skills:
      turn_to_deg(90, 100);                    // more accurate than wheel-deg
      drive_in(24, 120);
      turn_to_deg(0, 100);
      // If you still want a coarse wheel-based spin:
      turn_by_wheel_deg(720, 100);
      break;

    default: // DoNothing
      wait_ms(250);
      break;
  }
}

