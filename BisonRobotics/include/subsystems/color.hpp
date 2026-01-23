#pragma once
#include "mech.hpp"
#include "config/drive_select.hpp"

namespace color {
  void initialize();
  // Returns optical hue [0..360), proximity [0..255]
  double hue();
  int proximity();
  void set_led_pwm(int pct);      // 0..100
  bool is_team_color(TeamColor c);
}
