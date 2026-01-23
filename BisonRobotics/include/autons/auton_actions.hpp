#pragma once
#include <cstdint>

namespace auton_actions {

  // ===================== Movement primitives (blocking) =====================
  // Distances are inches, angles are degrees.
  void drive_inches(double inches, int vel = 100);
  void strafe_right_inches(double inches, int vel = 100);
  void strafe_left_inches(double inches, int vel = 100);

  // Open-loop “just push power for time”.
  // fwd/str/rot are -127..127
  void move_ms(uint32_t ms, int fwd, int str, int rot, bool field_centric = false);

  // IMU-based turns (more accurate than wheel-deg)
  void turn_to_deg(double target_deg, int maxVel = 100, double kP = 1.2);
  void turn_by_deg(double delta_deg, int maxVel = 100, double kP = 1.2);

  // Coarse drivetrain-geometry based spin (if you still want it)
  void turn_by_wheel_deg(double wheel_deg, int vel = 100);

  // Utility
  void wait_ms(uint32_t ms);

  // ===================== Subsystem helpers =====================
  // These should call into your real subsystem code (mech/intake/feeder/etc).
  // Keep these as your "auton-facing API" so routes never touch motors directly.

  void intake_on(int pct = 127);
  void intake_off();
  void intake_reverse(int pct = 127);

  void feeder_off();
  void feeder_intake_store();
  void feeder_score_lower();
  void feeder_score_middle();
  void feeder_score_upper();

  void loader_deploy();
  void loader_retract();

  void park_brake();
  void park_release();
}