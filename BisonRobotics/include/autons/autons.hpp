#pragma once
#include <cstdint>
#include "util/types.hpp"

// ===================== AUTON API =====================
namespace autons {

  // Entry point from autonomous()
  void run(AutonRoutine which);

  // ---- Movement primitives (blocking) ----
  // All distances are in inches, turn angles in degrees.
  void drive_in(double inches, int vel = 100);
  void strafe_right_in(double inches, int vel = 100);
  void strafe_left_in(double inches, int vel = 100);

  // IMU-based turns (preferred for accuracy)
  void turn_to_deg(double target_deg, int maxVel = 100, double kP = 1.2);
  void turn_by_deg(double delta_deg, int maxVel = 100, double kP = 1.2);

  // Wheel-degree turn (coarse; uses drivetrain geometry)
  void turn_by_wheel_deg(double wheel_deg, int vel = 100);

  // ---- Utility ----
  void wait_ms(uint32_t ms);

  // Optional timeouts (return true if finished within timeout)
  bool drive_in_timeout(double inches, int vel, uint32_t timeout_ms);
  bool strafe_right_in_timeout(double inches, int vel, uint32_t timeout_ms);

  // ---- Subsystem stubs (fill these in later) ----
  // Wire these to your real subsystems when ready.
  void intake_set(int pct);                       // -127..127
  void intake_for_ms(int pct, uint32_t ms);       // runs, then stops
  void lift_set(int pct);                         // example placeholder
  void lift_hold();                               // example placeholder
}
