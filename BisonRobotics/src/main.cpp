#include "main.h"
#include "xdrive.hpp"
#include "util/selector.hpp"   
#include "autons/autons.hpp"   
#include "pros/misc.h"
#include "mech.hpp"

using namespace pros;

static Controller master(E_CONTROLLER_MASTER);

// Global picks locked in competition_initialize()
static DriveMode    g_mode  = DriveMode::FieldCentric;
static AutonRoutine g_auton = AutonRoutine::DoNothing;

/** Runs once at boot */
void initialize() {
  lcd::initialize();
  lcd::print(0, "X-Drive Ready");
  
  mech::initialize();  

  xdrive::initialize();      // motors + IMU calibration
  selector::init();          // NEW: bring up the pre-match selector UI
  xdrive::start_telemetry(); // optional HUD on the Brain
}

/** Idle during disable */
void disabled() {}

/** Pre-match window: choose drive mode + auton */
void competition_initialize() {
  // Give yourself ~6s to pick. (You can also loop while competition::is_disabled())
  const uint32_t t0 = millis();
  while (millis() - t0 < 6000) {
    selector::ui_loop_once();  // D-pad L/R flips drive mode, A cycles auton
    delay(10);
  }

  // Lock selections for the match
  g_mode  = selector::drive_mode();
  g_auton = selector::auton();

  lcd::print(6, "Locked: %s | %s",
             selector::drive_mode_name(g_mode),
             selector::auton_name(g_auton));
}

/** Autonomous: delegate to your selected routine */
void autonomous() {
  autons::run(g_auton);
}

/** Driver control */
void opcontrol() {
  while (true) {
    // In-match A/B test: Y toggles Field <-> Robot
    if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)) {
      g_mode = (g_mode == DriveMode::FieldCentric)
                 ? DriveMode::RobotCentric
                 : DriveMode::FieldCentric;
      master.rumble("."); // haptic confirm
    }

    if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_A)) {
    xdrive::zero_field_forward();
    master.rumble(".."); // haptic confirm
    }

    mech::teleop(master); //buttons -> intake/feeder/color

    const int fwd = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);   // forward/back
    const int str = master.get_analog(E_CONTROLLER_ANALOG_LEFT_X);   // strafe
    const int rot = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);  // rotate

    xdrive::drive(fwd, str, rot, g_mode == DriveMode::FieldCentric);
    delay(10);
  }
}
