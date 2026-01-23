#pragma once
#include "api.h"
#include <cmath>
#include <algorithm>

namespace xdrive {

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

// Parking (hold position / resist pushing)
void set_park_enabled(bool enabled);
bool park_enabled();

// Convenience
inline double inches_to_deg(double inches, double wheel_diam_in = 4.0) {
  const double circ = wheel_diam_in * M_PI;
  return (inches / circ) * 360.0;
}

} // namespace xdrive
