#include "main.h"
#include "xdrive.hpp"
#include "util/selector.hpp"
#include "autons/auton_routes.hpp"
#include "autons/auton_actions.hpp"
#include "util/types.hpp"
#include "pros/misc.h"
#include "pros/llemu.hpp"
#include "mech.hpp"
//#include "odom_sys.hpp"
#include "subsystems/intake.hpp"

using namespace pros; //saves time in typing pros::

static AutonRoutine g_selected_auton = AutonRoutine::Red_Left; // "first auton" default

static Controller master(E_CONTROLLER_MASTER); // decalres single controller
static DriveMode    g_locked_drive = DriveMode::RobotCentric;
static AutonRoutine g_locked_auton = AutonRoutine::Red_Left;
static bool         g_is_locked    = false;

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
void disabled() {
}

// =================== Competition Init ===================
void competition_initialize() {

  // Lock selections once (these will be used for the whole match)
  //g_locked_drive = selector::drive_mode();
  //g_locked_auton = selector::auton();
  //g_is_locked = true;

  // Initialize odometry hardware and set starting pose
  // odom_sys::init_once_for_auton();
  // odom_sys::reset_pose({0.0, 0.0, 0.0});   // ← set your tile pose here

  // Brain: show ONLY the auton name (simple & obvious)
  lcd::clear();
  lcd::print(0, "AUTON: %s", selector::auton_name(g_locked_auton));
}

// =================== Autonomous =====================
void autonomous() {
  xdrive::set_park_enabled(false);

  // default
  // If for some reason we never ran comp init, fall back safely.
  //const AutonRoutine a = g_is_locked ? g_locked_auton : selector::auton();
      // competition locked pick overrides
  
  auton_routes::run(g_selected_auton);

}

// =================== Operator Control ===================
void opcontrol() {
  // Take the locked picks as our starting point
  DriveMode drive_mode = selector::drive_mode();

  // when not at comp
  bool ran_home_auton = false;

  while (true) {
    // In-match toggle Field<->Robot with Y
    if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)) {
      drive_mode = (drive_mode == DriveMode::FieldCentric)
        ? DriveMode::Robothttps://github.com/nicholas-moran-ndsu/Bison-Robotics-VEXU/pull/9/conflict?name=BisonRobotics%252Fsrc%252Fodom_sys.cpp&ancestor_oid=2fc298c48ef782f623fc175836851e7a87e3c818&base_oid=4fd144c9f166d469233054a2e350c78e40961c95&head_oid=0a6b47969e4bd54ee5402ec0080287a88014c253Centric
        : DriveMode::FieldCentric;
      master.rumble(".");
    }

    // A toggles Park mode (drivetrain hold)
    if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_A)) {
      const bool next = !xdrive::park_enabled();
      xdrive::set_park_enabled(next);

      // haptic: 1 pulse = park ON, 2 pulses = park OFF
      master.rumble(next ? "." : "..");
    }

    // resets heading to zero by pressing B
    //if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_B)) {
    //xdrive::zero_field_forward(); 
    //master.rumble(".."); // haptic confirm
    //}

    if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_B)) {
      // Toggle alliance / team color
      const TeamColor next =
        (mech::team_color() == TeamColor::Blue) ? TeamColor::Red : TeamColor::Blue;

      mech::set_team_color(next);

      // Optional: quick feedback
      master.rumble(next == TeamColor::Blue ? "." : "..");   // 1 pulse=Blue, 2=Red
      pros::lcd::print(1, "TEAM: %s", next == TeamColor::Blue ? "BLUE" : "RED");
    }


    // ================= HOME TEST AUTON TRIGGER =================
    // Without field control, autonomous() is never called.
    // So allow running a default auton manually.
    if (!pros::competition::is_connected() && !ran_home_auton) {
      // Hold A + B for 300ms to start auton (hard to hit accidentally)
      if (master.get_digital(E_CONTROLLER_DIGITAL_A) &&
          master.get_digital(E_CONTROLLER_DIGITAL_B)) {

        pros::delay(300);
        if (master.get_digital(E_CONTROLLER_DIGITAL_A) &&
            master.get_digital(E_CONTROLLER_DIGITAL_B)) {

          ran_home_auton = true;

          // Safety: stop driver commands and un-park
          xdrive::set_park_enabled(false);
          xdrive::drive(0, 0, 0, false);

          // Run your "first auton" (or whatever default you want)
          auton_routes::run(g_selected_auton);

          // Stop everything after auton
          xdrive::drive(0, 0, 0, false);
          mech::feeder_stop();
          mech::intake_stop();
        }
      }
    }
    // ===========================================================

    mech::teleop(master); //buttons -> intake/feeder/color

    // robot-centric drive
    const int fwd = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);   // forward/back
    const int str = master.get_analog(E_CONTROLLER_ANALOG_LEFT_X);   // strafe
    const int rot = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);  // rotate

    //xdrive
    xdrive::drive(fwd, str, rot, drive_mode == DriveMode::RobotCentric);

    // Optional alive print every 500 ms
    static int tick = 0;
    if ((++tick % 50) == 0) { // about every 500 ms
      printf("[OP] alive t=%u ms\n", (unsigned)pros::millis());
      fflush(stdout);
    }

    pros::delay(10);
  }
}

// =======================================================
//                   CONTROLLER MAPPING
// =======================================================
//
// ---------------------- Driving -------------------------
// Left Stick  -> Translation (fwd/back, strafe)
// Right Stick -> Rotation
//
// Y (new press) -> Toggle Field-Centric / Robot-Centric
//   • Rumble "."  (1 pulse) = mode toggled
//
// B (new press) -> Zero heading / re-align field forward
//   • Rumble ".." (2 pulses) = heading zeroed
//
// A (new press) -> PARK toggle (drivetrain hold / lockout)
//   • Rumble "."  (1 pulse) = Park ON
//   • Rumble ".." (2 pulses) = Park OFF
//
// ---------------------- Intake --------------------------
// L1 (new press) -> Intake ON/OFF toggle
//   • (no rumble)
// L2 (new press) -> Intake direction flip
//   • Rumble "."  (1 pulse) = Forward (intaking)
//   • Rumble ".." (2 pulses) = Reverse (outtaking)
//
// ---------------------- Feeder --------------------------
// D-Pad RIGHT (new press) -> Intake / Store (toggle)
// D-Pad DOWN  (new press) -> Score LOWER   (toggle)
// D-Pad LEFT  (new press) -> Score MIDDLE  (toggle)
// D-Pad UP    (new press) -> Score UPPER   (toggle)
//
// X (new press) -> FORCE OFF (immediate stop + clears autosort overrides)
//   • (no rumble)
//
// Feeder Toggle Behavior:
// • Tap a D-pad direction → feeder switches to that mode
// • Tap the same direction again → feeder OFF
// • Tap a different direction → switch to new mode
// • Press X → feeder OFF no matter what
//
// Auto-Sort Behavior:
// • Only active during Intake/Store mode
// • Will temporarily override feeder motor2 to eject enemy color
// • X clears overrides immediately
//
// ---------------------- Loader (Pneumatics) -------------
// R1 (new press) -> Deploy loader
//   • (no rumble)
// R2 (new press) -> Retract loader
//   • (no rumble)
//
// --------------------------------------------------------
// Notes:
// • Intake is independent from feeder/loader
// • Park mode ignores joystick drive commands while enabled
// • Keep this block updated if you add any new controls
// =======================================================

