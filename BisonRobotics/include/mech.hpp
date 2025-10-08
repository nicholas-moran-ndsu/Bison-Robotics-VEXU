#pragma once
#include "subsystems/config.hpp"

namespace mech {
  void initialize();

  // Call every loop in opcontrol() to handle buttons + auto-sort
  void teleop(pros::Controller& master);

  // --- Direct controls you can call from auton ---
  void intake_set(int pct);           // -127..127
  void feeder_set(int pct);           // -127..127
  void intake_stop();
  void feeder_stop();

  // Color & sorting
  void set_team_color(TeamColor c);
  TeamColor team_color();
  void turn_on_auto_sort(bool on);
  bool auto_sort_enabled();
}
