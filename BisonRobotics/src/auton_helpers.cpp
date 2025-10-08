#include "autons/autons.hpp"
#include "xdrive.hpp"
#include "pros/apix.h"
#include <cmath>
#include <algorithm>

using namespace pros;

namespace {

// Keep units consistent
constexpr double kWheelDiamIn = 4.0;     // ======== SET WHEEL DIAMETER ========
constexpr double kHeadingEps  = 2.0;     // acceptable heading error (degrees)
constexpr uint32_t kTurnTimeoutMs = 2000;

// Wrap angle to (-180, 180]
double wrap180(double a) {
  while (a <= -180.0) a += 360.0;
  while (a >   180.0) a -= 360.0;
  return a;
}

} // anonymous namespace

namespace autons {

// ---------------- Movement primitives ----------------
void drive_in(double inches, int vel) {
  const double deg = xdrive::inches_to_deg(inches, kWheelDiamIn);
  xdrive::drive_forward_deg(deg, vel);
}

void strafe_right_in(double inches, int vel) {
  const double deg = xdrive::inches_to_deg(inches, kWheelDiamIn);
  xdrive::strafe_right_deg(deg, vel);
}

void strafe_left_in(double inches, int vel) {
  // left is right with negative distance
  strafe_right_in(-inches, vel);
}

// IMU-based proportional turn to absolute field heading (0..360)
void turn_to_deg(double target_deg, int maxVel, double kP) {
  const uint32_t t_deadline = pros::millis() + kTurnTimeoutMs;

  // Normalize target to [0, 360)
  while (target_deg < 0)   target_deg += 360.0;
  while (target_deg >= 360) target_deg -= 360.0;

  while (pros::millis() < t_deadline) {
    const double cur    = xdrive::heading_deg();        // 0..360 or 0 if IMU not ready
    const double error  = wrap180(target_deg - cur);    // -180..180

    if (std::fabs(error) < kHeadingEps) break;

    // P-control → rotation command in [-127, 127]
    int rot = (int)std::round(std::clamp(kP * error, -127.0, 127.0));
    // Cap by maxVel to keep turns gentle if desired
    rot = std::clamp(rot, -maxVel, maxVel);

    // Rotate in place (robot-centric). No translation.
    xdrive::drive(0, 0, rot, /*field_centric=*/false);
    pros::delay(10);
  }
  xdrive::drive(0, 0, 0, false); // stop
}

// Turn by relative delta (e.g., +90 means rotate CW 90°)
void turn_by_deg(double delta_deg, int maxVel, double kP) {
  const double cur   = xdrive::heading_deg();
  double target = cur + delta_deg;
  // Normalize to [0, 360) to keep the math stable
  while (target < 0)   target += 360.0;
  while (target >= 360) target -= 360.0;
  turn_to_deg(target, maxVel, kP);
}

// Wheel-degree based (coarse)
void turn_by_wheel_deg(double wheel_deg, int vel) {
  xdrive::turn_cw_deg(wheel_deg, vel);
}

// ---------------- Utility ----------------
void wait_ms(uint32_t ms) { pros::delay(ms); }

bool drive_in_timeout(double inches, int vel, uint32_t timeout_ms) {
  const uint32_t t0 = pros::millis();
  drive_in(inches, vel); // blocking helper
  return (pros::millis() - t0) <= timeout_ms;
}

bool strafe_right_in_timeout(double inches, int vel, uint32_t timeout_ms) {
  const uint32_t t0 = pros::millis();
  strafe_right_in(inches, vel);
  return (pros::millis() - t0) <= timeout_ms;
}

// ---------------- Subsystem stubs ----------------
// Replace these with real implementations when subsystems exist.
void intake_set(int pct) {
  // TODO: call your intake motor(s) here (e.g., intake.move(pct));
  (void)pct; // suppress unused warning until wired
}

void intake_for_ms(int pct, uint32_t ms) {
  intake_set(pct);
  pros::delay(ms);
  intake_set(0);
}

void lift_set(int pct) {
  // TODO: call your lift motor(s) here
  (void)pct;
}

void lift_hold() {
  // TODO: set brake mode/hold or closed-loop position hold
}

} // namespace autons
