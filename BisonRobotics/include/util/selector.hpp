#pragma once
#include <cstdint>
#include "util/types.hpp"
#include "pros/misc.h"
#include "pros/apix.h"


namespace selector {
  // Boot-time setup (no lcd::initialize here!)
  void init(pros::Controller& master);

  // Pre-match poll: handles button presses + updates screens
  void ui_loop_once(pros::Controller& master);

  // Access current selections (pre-lock)
  DriveMode    drive_mode();
  AutonRoutine auton();

  // Pretty names
  const char* drive_mode_name(DriveMode m);
  const char* auton_name(AutonRoutine a);
}