#include "util/selector.hpp"
#include "pros/llemu.hpp"   // Brain LCD interface
#include "pros/misc.h"      // Controller + buttons
#include "pros/apix.h"      // Includes all PROS API headers

using namespace pros;  // convenience alias for pros::

// ============================================================================
//  Selector Module
//  ----------------
//  Handles all pre-match UI logic for selecting drive mode and auton routine.
//  - User interacts via Controller (D-pad and buttons).
//  - Displays current selections on both Brain LCD and Controller screen.
//  - Stores the choices internally (static variables) until locked in main.
// ============================================================================

// Default drive mode and auton (used if no selection is made before match)
static DriveMode    g_mode  = DriveMode::FieldCentric;
static AutonRoutine g_auton = AutonRoutine::Red_Left;

// List of human-readable auton names.
// ⚠️ The order *must* match the AutonRoutine enum defined in types.hpp.
static const char* AUTON_NAMES[] = {
  "Red_Left",
  "Red_Right",
  "Blue_Left",
  "Blue_Right",
  "Skills"
};

// Number of auton options (computed automatically)
static constexpr int kAutonCount =
  static_cast<int>(sizeof(AUTON_NAMES) / sizeof(AUTON_NAMES[0]));

// Compile-time safety check: enum and name array must stay aligned.
// If you add an auton to the enum but forget to update this array,
// this assertion will fail at compile time.
static_assert(static_cast<int>(AutonRoutine::Skills) == kAutonCount - 1,
              "Update AUTON_NAMES or AutonRoutine enum order!");

// Converts a DriveMode enum to a display string.
const char* selector::drive_mode_name(DriveMode m) {
  return (m == DriveMode::FieldCentric) ? "Field-Centric" : "Robot-Centric";
}

// Converts an AutonRoutine enum to a display string.
const char* selector::auton_name(AutonRoutine a) {
  int i = static_cast<int>(a);
  return (i >= 0 && i < kAutonCount) ? AUTON_NAMES[i] : "?";
}

// -----------------------------------------------------------------------------
// init()
// Called once from main.cpp (after pros::lcd::initialize()) to display usage
// hints and current defaults on both the Brain screen and Controller.
// -----------------------------------------------------------------------------
void selector::init(Controller & master) {
  // Brain LCD quick instructions
  lcd::print(0, "Selector ready");
  lcd::print(1, "L/R: Drive   A: Auton");
  lcd::print(2, "Y toggles drive in teleop");

  // Controller hints
  master.clear();
  master.print(0, 0, "< > drive   A auton");
  master.print(1, 0, "Y toggles in teleop");

  // Show the current default values
  lcd::print(4, "Drive: %s", drive_mode_name(g_mode));
  lcd::print(5, "Auton: %s", auton_name(g_auton));
  master.print(2, 0, "D:%s A:%s          ",
               drive_mode_name(g_mode),
               auton_name(g_auton));
}

// -----------------------------------------------------------------------------
// ui_loop_once()
// Called repeatedly during competition_initialize().
// Each call polls for new button presses and updates the displays accordingly.
// Use get_digital_new_press() so each button press is registered only once.
// -----------------------------------------------------------------------------
void selector::ui_loop_once(Controller & master) {
  // ---- Drive mode selection (D-pad left/right) ----
  const bool flipDrive =
      master.get_digital_new_press(E_CONTROLLER_DIGITAL_LEFT) ||
      master.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT);

  if (flipDrive) {
    // Toggle between Field and Robot centric
    g_mode = (g_mode == DriveMode::FieldCentric)
               ? DriveMode::RobotCentric
               : DriveMode::FieldCentric;
  }

  // ---- Auton selection (A button cycles) ----
  if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_A)) {
    // Increment auton index and wrap around at the end
    int next = static_cast<int>(g_auton) + 1;
    g_auton = static_cast<AutonRoutine>(next % kAutonCount);
  }

  // ---- Display updates (Brain + Controller) ----
  lcd::print(4, "Drive: %s", drive_mode_name(g_mode));
  lcd::print(5, "Auton: %s", auton_name(g_auton));

  // Controller line 2 (0-based) combines both values in one line
  master.print(2, 0, "D:%s A:%s          ",
               drive_mode_name(g_mode),
               auton_name(g_auton));
}


// -----------------------------------------------------------------------------
// Accessors (used by main.cpp to read selections)
// -----------------------------------------------------------------------------
DriveMode    selector::drive_mode() { return g_mode; }
AutonRoutine selector::auton()      { return g_auton; }
