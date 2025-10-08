#include "mech.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/feeder.hpp"
#include "subsystems/color.hpp"
#include "pros/apix.h"

namespace {
  TeamColor g_team = TeamColor::Red;
  bool g_autoSort = false;

  // tiny helper: reject if wrong color detected
  void auto_sort_tick() {
    if (!g_autoSort) return;
    if (color::proximity() < PROX_THRESHOLD) return;

    const bool ours = color::is_team_color(g_team);
    if (!ours) {
      // brief reverse to spit out wrong color without blocking teleop feel
      feeder::set(-80);
      intake::set(-80);
      pros::delay(120);
      feeder::set(0);
      intake::set(0);
    }
  }
}

void mech::initialize() {
  intake::initialize();
  feeder::initialize();
  color::initialize();
}

void mech::teleop(pros::Controller& master) {
  // Intake controls
  if (master.get_digital(BTN_INTAKE_IN))      intake::set(127);
  else if (master.get_digital(BTN_INTAKE_OUT))intake::set(-127);
  else                                        intake::stop();

  // Feeder controls
  if (master.get_digital(BTN_FEED_INDEX))     feeder::set(127);
  else if (master.get_digital(BTN_FEED_REVERSE)) feeder::set(-127);
  else                                        feeder::stop();

  // Toggle auto-sort
  if (master.get_digital_new_press(BTN_SORT_TOGGLE)) {
    g_autoSort = !g_autoSort;
    master.rumble(g_autoSort ? "." : "-"); // haptic confirm
  }

  auto_sort_tick(); // non-blocking quick check each loop
}

// --- Auton-friendly wrappers ---
void mech::intake_set(int pct) { intake::set(pct); }
void mech::feeder_set(int pct) { feeder::set(pct); }
void mech::intake_stop()       { intake::stop();   }
void mech::feeder_stop()       { feeder::stop();   }

void mech::set_team_color(TeamColor c) { g_team = c; }
TeamColor mech::team_color()           { return g_team; }

void mech::turn_on_auto_sort(bool on)  { g_autoSort = on; }
bool mech::auto_sort_enabled()         { return g_autoSort; }
