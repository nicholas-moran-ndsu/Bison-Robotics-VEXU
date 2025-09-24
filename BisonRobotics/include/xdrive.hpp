#pragma once
#include "api.h"

namespace xdrive {

// ====== CONFIGURE THESE ======
constexpr int PORT_FL = 1;   // Front-Left  motor port
constexpr int PORT_FR = 2;   // Front-Right motor port
constexpr int PORT_BL = 3;   // Back-Left   motor port
constexpr int PORT_BR = 4;   // Back-Right  motor port

constexpr bool REVERSED_FL = false;
constexpr bool REVERSED_FR = true;
constexpr bool REVERSED_BL = false;
constexpr bool REVERSED_BR = true;

constexpr pros::motor_gearset_e_t  GEARSET       = pros::E_MOTOR_GEARSET_18;
constexpr pros::motor_encoder_units_e_t ENCODERS = pros::E_MOTOR_ENCODER_DEGREES;

// IMU (optional, for field-centric). Set to -1 to disable.
constexpr int IMU_PORT = -1; //5

// Control options
constexpr int  DEADBAND = 5;   // joystick deadband (0–127)
constexpr bool SQUARE_INPUTS = true; // exponential feel for finer low-speed control

// Init / utilities
void initialize();
double heading_deg(); // returns 0..360 if IMU present, otherwise 0

// Teleop drive
// fwd: forward/back   (+forward), str: strafe right/left (+right), rot: rotate CW/CCW (+CW)
// All in joystick range [-127..127]. If field_centric=true, IMU is used to rotate the (fwd,str) vector.
void drive(int fwd, int str, int rot, bool field_centric = false);

// Simple blocking moves (open-loop). Uses built-in encoders; tune speeds for your bot.
// NOTE: These are basic helpers for skills prototyping—not precise motion profiles.
void drive_forward_deg(double wheel_deg, int speed = 100);   // +deg forward, -deg backward
void strafe_right_deg(double wheel_deg, int speed = 100);    // +deg right,  -deg left
void turn_cw_deg(double wheel_deg, int speed = 100);         // +deg CW,     -deg CCW

// Convenience: convert inches to wheel degrees (assuming 1:1, set your wheel diameter)
inline double inches_to_deg(double inches, double wheel_diam_in = 4.0) {
  const double circ = wheel_diam_in * 3.14159265358979323846;
  return (inches / circ) * 360.0;
}

// Start/stop a background LCD telemetry task for the four drive motors.
void start_telemetry();
void stop_telemetry();

} // namespace xdrive
