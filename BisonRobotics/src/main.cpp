#include "main.h"
#include "xdrive.hpp"
#include "util/selector.hpp"
#include "autons/autons.hpp"
#include "pros/misc.h"
#include "pros/llemu.hpp"
#include "mech.hpp"

using namespace pros; //saves time in typing pros::

static Controller master(E_CONTROLLER_MASTER); // decalres single controller

/** Runs once at boot */
void initialize() {
  lcd::initialize();
  
  mech::initialize();  
  xdrive::initialize();      // motors + IMU calibration
  selector::init(master);          // bring up the pre-match selector UI
  //xdrive::start_telemetry(); // optional HUD on the Brain
}

/** Idle during disable */
void disabled() {}

/** Pre-match window: choose drive mode + auton */
void competition_initialize() {
  // Give yourself ~6s to pick. (You can also loop while competition::is_disabled())
  const uint32_t t0 = millis();
  while (millis() - t0 < 6000) {
    selector::ui_loop_once(master);  // D-pad L/R flips drive mode, A cycles auton
    delay(10);
  }

  // Lock (just report what was picked)
  lcd::print(6, "Locked: %s | %s",
             selector::drive_mode_name(selector::drive_mode()),
             selector::auton_name(selector::auton()));
}

/** Autonomous: delegate to your selected routine */
void autonomous() {
  autons::run(selector::auton());
}

/** Driver control */
void opcontrol() {
  // Take the locked picks as our starting point
  DriveMode drive_mode = selector::drive_mode();

  while (true) {
    // In-match toggle Field<->Robot with Y
    if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)) {
      drive_mode = (drive_mode == DriveMode::FieldCentric)
                     ? DriveMode::RobotCentric
                     : DriveMode::FieldCentric;
      master.rumble(".");
    }

    // resets heading to zero by pressing B
    if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_B)) {
    xdrive::zero_field_forward(); 
    master.rumble(".."); // haptic confirm
    }

    mech::teleop(master); //buttons -> intake/feeder/color

    // robot-centric drive
    const int fwd = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);   // forward/back
    const int str = master.get_analog(E_CONTROLLER_ANALOG_LEFT_X);   // strafe
    const int rot = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);  // rotate

    //xdrive
    xdrive::drive(fwd, str, rot, drive_mode == DriveMode::FieldCentric);

    delay(10);
  }
}