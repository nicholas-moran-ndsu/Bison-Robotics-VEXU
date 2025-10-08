#include "subsystems/color.hpp"
#include "pros/apix.h"

namespace { pros::Optical optical(PORT_OPTICAL); }

void color::initialize() {
  optical.set_led_pwm(100); // full illumination for reliable hue
}

double color::hue()        { return optical.get_hue(); }
int    color::proximity()  { return optical.get_proximity(); }
void   color::set_led_pwm(int pct) { optical.set_led_pwm(pct); }

bool color::is_team_color(TeamColor c) {
  const double h = hue();
  if (proximity() < PROX_THRESHOLD) return false; // too far/uncertain
  if (c == TeamColor::Red)  return (h >= HUE_RED_MIN  && h <= HUE_RED_MAX);
  if (c == TeamColor::Blue) return (h >= HUE_BLUE_MIN && h <= HUE_BLUE_MAX);
  return false;
}
