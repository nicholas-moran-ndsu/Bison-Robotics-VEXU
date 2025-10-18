#pragma once

#ifdef SIM
namespace pros {
  enum motor_gearset_e_t { E_MOTOR_GEARSET_18 = 0 };
  enum motor_encoder_units_e_t { E_MOTOR_ENCODER_DEGREES = 0 };
}
#else
#include "api.h"
#endif

#include <cmath>
#include <algorithm>

namespace xdrive {

// ====== CONFIGURE THESE ======
constexpr int PORT_FL = 2;   // Front-Left  motor port
constexpr int PORT_FR = 10;   // Front-Right motor port
constexpr int PORT_BL = 11;   // Back-Left   motor port
constexpr int PORT_BR = 20;   // Back-Right  motor port

constexpr bool REVERSED_FL = false;
constexpr bool REVERSED_FR = true;
constexpr bool REVERSED_BL = false;
constexpr bool REVERSED_BR = true;

/*
E_MOTOR_GEARSET_36  → Red cartridge (100 RPM, high torque)
E_MOTOR_GEARSET_18  → Green cartridge (200 RPM, standard)
E_MOTOR_GEARSET_06  → Blue cartridge (600 RPM, high speed)
*/
constexpr pros::motor_gearset_e_t GEARSET = pros::E_MOTOR_GEARSET_18; //green cart
/*
E_MOTOR_ENCODER_DEGREES   → motor shaft position in degrees
E_MOTOR_ENCODER_ROTATIONS → position in full revolutions
E_MOTOR_ENCODER_COUNTS    → raw internal counts (ticks)
*/
constexpr pros::motor_encoder_units_e_t ENCODERS = pros::E_MOTOR_ENCODER_DEGREES;

// IMU (optional, for field-centric). Set to -1 to disable.
constexpr int IMU_PORT = 16; // =========== SET PORT ===========

// Control options
constexpr int  DEADBAND = 2;           // joystick deadband (0..127)
constexpr double DRIVE_EXPONENT = 2.5; // Drive response curve 

// Init / utilities
void initialize();
double heading_deg(); // 0..360 if IMU present, else 0

// Teleop drive (joystick units -127..127)  +fwd, +right, +CW
void drive(int fwd, int str, int rot, bool field_centric = false);

void zero_field_forward(); // set current IMU angle as field-forward (0°)

// Simple blocking helpers (no-ops in SIM)
void drive_forward_deg(double wheel_deg, int speed = 100);
void strafe_right_deg(double wheel_deg, int speed = 100);
void turn_cw_deg(double wheel_deg, int speed = 100);

// Convenience
inline double inches_to_deg(double inches, double wheel_diam_in = 4.0) {
  const double circ = wheel_diam_in * M_PI;
  return (inches / circ) * 360.0;
}

// LCD telemetry (does nothing in SIM)
void start_telemetry();
void stop_telemetry();

} // namespace xdrive
