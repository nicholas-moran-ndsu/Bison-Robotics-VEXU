#pragma once
#include "config/drive_select.hpp"
#include "subsystems/feeder.hpp"

namespace mech {
  void initialize();

  // Call every loop in opcontrol() to handle buttons + auto-sort
  void teleop(pros::Controller& master);

  // Call frequently in auton and teleop to run auto-sort state machine
  void update();

  // --- Direct controls you can call from auton ---
  void intake_set(int pct);           // -127..127
  void intake_stop();

  void set_feeder_mode(feeder::Mode m);   // sets internal mode + applies to motors
  feeder::Mode feeder_mode();             // read current mode
  void feeder_stop();

  // Color & sorting
  void set_team_color(TeamColor c);
  TeamColor team_color();

  // Loader pneumatics
  void loader_deploy();
  void loader_retract();
  bool loader_is_deployed();
} 
