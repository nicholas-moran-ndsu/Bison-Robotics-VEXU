#include "main.h"
#include "xdrive.hpp"
#include "pros/misc.h"

using namespace pros;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		lcd::set_text(2, "I was pressed!");
	} else {
		lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	lcd::initialize();
	lcd::print(0, "X-Drive Ready");

	xdrive::initialize();  // calibrates IMU if configured
	xdrive::start_telemetry();   // <-- start screen updates
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	// Move ~24 inches forward (4" wheel default)
  xdrive::drive_forward_deg(xdrive::inches_to_deg(24.0), 100);
  delay(300);
  // Strafe right 12 inches
  xdrive::strafe_right_deg(xdrive::inches_to_deg(12.0), 100);
  delay(300);
  // Turn ~360 wheel degrees per side for a spin (tune!)
  xdrive::turn_cw_deg(720, 100);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	Controller master(pros::E_CONTROLLER_MASTER);
	const bool field = true; // toggle to enable field-centric (requires IMU)

	while (true) {
		int fwd = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);   // forward/back
		int str = master.get_analog(E_CONTROLLER_ANALOG_LEFT_X);   // strafe
		int rot = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);  // rotate
		xdrive::drive(fwd, str, rot, field);
		delay(10);
	}
}