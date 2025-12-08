#include "main.h"
#include "xdrive.hpp"
#include "util/selector.hpp"
#include "autons/autons.hpp"
#include "pros/misc.h"
#include "pros/llemu.hpp"
#include "mech.hpp"
#include "odom_sys.hpp"

using namespace pros; //saves time in typing pros::

static Controller master(E_CONTROLLER_MASTER); // decalres single controller

// ================== Initialization ===================
void initialize() {
  lcd::initialize();
  mech::initialize();  
  xdrive::initialize();        // motors + IMU calibration
  selector::init(master);      // bring up the pre-match selector UI
  //xdrive::start_telemetry(); // optional HUD on the Brain

  printf("[INIT] hello from initialize()\n");
  fflush(stdout);
}

// ================== Disabled ===================
void disabled() {}

// =================== Competition Init ===================
void competition_initialize() {
  // Give yourself ~6s to pick. (You can also loop while competition::is_disabled())
  const uint32_t t0 = millis();
  // Poll selector UI until match starts or time runs out
  while (competition::is_disabled() && millis() - t0 < 6000)  {
    selector::ui_loop_once(master);  // D-pad L/R flips drive mode, A cycles auton
    delay(10);
  }

  // Initialize odometry hardware and set starting pose
  odom_sys::init_once_for_auton();
  odom_sys::reset_pose({0.0, 0.0, 0.0});   // ← set your tile pose here

  // Lock (just report what was picked)
  lcd::print(6, "Locked: %s | %s",
  selector::drive_mode_name(selector::drive_mode()),
  selector::auton_name(selector::auton()));
}

// =================== Autonomous =====================
void autonomous() {
  // Run your selected autonomous routine
  autons::run(selector::auton());
}

// =================== Operator Control ===================
void opcontrol() {
  // Take the locked picks as our starting point
  DriveMode drive_mode = selector::drive_mode();

  // Make sure odom is created and sensors are zeroed when driver control starts.
  odom_sys::init_once_for_auton();
  // Start the 100 Hz updater task (or do manual updates in this loop)
  odom_sys::start_updater_task();
  // Optional: start a raw data logger task
  odom_sys::start_raw_logger_task();

  // (optional) One-time banner so you know opcontrol is active
  printf("[OP] odom updater + raw logger started\n"); fflush(stdout);

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

    // Optional alive print every 500 ms
    static int tick = 0;
    if ((++tick % 50) == 0) { // about every 500 ms
      printf("[OP] alive t=%u ms\n", (unsigned)pros::millis());
      fflush(stdout);
    }

    pros::delay(10);
  }
}