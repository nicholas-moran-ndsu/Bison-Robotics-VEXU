#include "util/selector.hpp"
#include "pros/apix.h"   // PROS LCD + Controller APIs
#include <cstddef>       // std::size_t for array size if needed

using namespace pros;

// ------------------------ Internal state ------------------------
// Defaults for power-on: Field-centric drive and "DoNothing" auton.
static DriveMode    g_mode  = DriveMode::FieldCentric;
static AutonRoutine g_auton = AutonRoutine::DoNothing;

// ------------------------ Naming helpers ------------------------
// Human-readable names for UI/debug. We keep an array of auton names
// whose order MUST match the enum order (DoNothing=0, NearSide=1, ...).
// This makes it trivial to add new autons later by appending to both.
static const char* AUTON_NAMES[] = {"DoNothing", "Near", "Far", "Skills"};

// Number of entries (compile-time). If you add more names above,
// this updates automatically and keeps wrap-around correct.
static constexpr int kAutonCount =
  static_cast<int>(sizeof(AUTON_NAMES) / sizeof(AUTON_NAMES[0]));

// Optional sanity check: verifies the last enum value equals the last
// index in AUTON_NAMES. If you add a new enum at the end but forget
// to update AUTON_NAMES, this assert will remind you at compile time.
static_assert(static_cast<int>(AutonRoutine::Skills) == kAutonCount - 1,
              "Update AUTON_NAMES or the AutonRoutine enum order!");

const char* selector::drive_mode_name(DriveMode m) {
  return (m == DriveMode::FieldCentric) ? "Field-Centric" : "Robot-Centric";
}

const char* selector::auton_name(AutonRoutine a) {
  const int i = static_cast<int>(a);
  // Defensive guard: if enum and names ever get out of sync, avoid UB.
  if (i < 0 || i >= kAutonCount) return "?";
  return AUTON_NAMES[i];
}


// ------------------------ UI lifecycle ------------------------
// Call once at boot. Brings up Brain LCD and prints usage hints.
void selector::init() {
  lcd::initialize();
  lcd::print(0, "Selector ready");
  lcd::print(1, "L/R: Drive   A: Auton");
  lcd::print(2, "Y toggles drive in teleop");

  // Controller hints
  static pros::Controller master(pros::E_CONTROLLER_MASTER);
  master.clear();
  master.print(0, 0, "< > drive   A auton");
  master.print(1, 0, "Y toggles in teleop");

  // Optional: show the defaults immediately
  lcd::print(4, "Drive: %s", selector::drive_mode_name(g_mode));
  lcd::print(5, "Auton: %s", selector::auton_name(g_auton));
  master.print(2, 0, "D:%s A:%s          ",
               selector::drive_mode_name(g_mode),
               selector::auton_name(g_auton));
}

// Poll once and refresh the screens. Call this repeatedly during
// competition_initialize() (or any pre-enable loop) e.g. every 10ms.
void selector::ui_loop_once() {
  static pros::Controller master(pros::E_CONTROLLER_MASTER);

  // ---- Drive mode select (D-Pad L/R) ----
  const bool flipDrive =
      master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT) ||
      master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT);

  if (flipDrive) {
    g_mode = (g_mode == DriveMode::FieldCentric)
               ? DriveMode::RobotCentric
               : DriveMode::FieldCentric;
  }

  // ---- Auton select (A button cycles) ----
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
    int next = static_cast<int>(g_auton) + 1;
    g_auton = static_cast<AutonRoutine>(next % kAutonCount);  // wrap-around
  }

  // ---- Brain LCD ----
  pros::lcd::print(4, "Drive: %s", selector::drive_mode_name(g_mode));
  pros::lcd::print(5, "Auton: %s", selector::auton_name(g_auton));

  // ---- Controller (0–2 only) ----
  // Put both on line 2; pad with spaces to clear leftovers.
  master.print(2, 0, "D:%s A:%s          ",
               selector::drive_mode_name(g_mode),
               selector::auton_name(g_auton));
  // (No line 3 prints!)
}

// ------------------------ Accessors ------------------------
DriveMode    selector::drive_mode() { return g_mode; }
AutonRoutine selector::auton()      { return g_auton; }

// ------------------------ Extending the selector ------------------------
/*
How to add more autonomous routines later:

1) In selector.hpp, append a new enumerator at the END, e.g.:
     enum class AutonRoutine : uint8_t { DoNothing, NearSide, FarSide, Skills, CenterRush };

2) In this file:
   - Append "CenterRush" to AUTON_NAMES in the SAME order.
   - You do NOT need to change kAutonCount; it's computed from the array.
   - The static_assert will continue to verify alignment.

3) In your autons.cpp switch, add a case:
     case AutonRoutine::CenterRush: /* ... * / break;

Optional: If you prefer not to rely on enum order, you can store a
vector/array of {AutonRoutine, const char* name} pairs and cycle the
*index* over that list instead of casting integers, but the simple
order-matching approach above is perfectly fine for VRC/VEXU robots.
*/

/*
Key-repeat behavior (optional):

If you WANT holding a button to keep cycling (instead of single taps),
replace get_digital_new_press(...) with get_digital(...), and add a
repeat delay/timer so it doesn't scroll too fast. Example skeleton:

  static uint32_t lastRepeat = 0;
  const bool held = master.get_digital(E_CONTROLLER_DIGITAL_A);
  if (held && (millis() - lastRepeat > 200)) {
    // advance selection
    lastRepeat = millis();
  }

Edge-triggered new_press is recommended on-field to prevent accidental
multiple changes due to button bounce or stress taps.
*/
