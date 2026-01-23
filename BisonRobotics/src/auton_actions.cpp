#include "autons/auton_actions.hpp"
#include "xdrive.hpp"
#include "config/drive_select.hpp"
#include "mech.hpp"
#include "subsystems/feeder.hpp" 
#include "pros/misc.h"
#include <cmath>
#include <algorithm>

namespace {
  // inches -> wheel degrees (geometry only)
  inline double inches_to_wheel_deg(double inches) {
    // wheel_deg = 360 * inches / (pi * D)
    return 360.0 * (inches / (M_PI * WHEEL_DIAMETER_IN));
  }

  // Wrap angle to (-180, 180]
  inline double wrap180(double a) {
    while (a <= -180.0) a += 360.0;
    while (a >   180.0) a -= 360.0;
    return a;
  }

  constexpr double   kHeadingEpsDeg = 2.0;
  constexpr uint32_t kTurnTimeoutMs = 2000;
  constexpr uint32_t kLoopDtMs      = 10;
}

namespace auton_actions {

// ---------------- Movement primitives (blocking) ----------------
void drive_inches(double inches, int vel) {
  xdrive::drive_forward_deg(inches_to_wheel_deg(inches), vel);
  //mech::update();
}

void strafe_right_inches(double inches, int vel) {
  xdrive::strafe_right_deg(inches_to_wheel_deg(inches), vel);
  //mech::update();
}

void strafe_left_inches(double inches, int vel) {
  strafe_right_inches(-inches, vel);
  //mech::update();
}

void move_ms(uint32_t ms, int fwd, int str, int rot, bool field_centric) {
  xdrive::drive(fwd, str, rot, field_centric);

  // Keep sorter alive during timed open-loop moves
  const uint32_t t0 = pros::millis();
  while (pros::millis() - t0 < ms) {
    //mech::update();
    pros::delay(10);
  }

  xdrive::drive(0, 0, 0, field_centric);
}

void park_brake() {
  // Engage “hold” brake and ignore drive commands (xdrive::drive does nothing while parked)
  xdrive::set_park_enabled(true);

  // Optional: a few ticks so autosort can run and the brake mode fully applies
  //mech::update();
  pros::delay(20);
  mech::update();
}

void park_release() {
  xdrive::set_park_enabled(false);
}

// ---------------- Turns ----------------
// IMU-based proportional turn to absolute heading [0..360)
void turn_to_deg(double target_deg, int maxVel, double kP) {
  const uint32_t deadline = pros::millis() + kTurnTimeoutMs;

  // Normalize target to [0, 360)
  while (target_deg < 0) target_deg += 360.0;
  while (target_deg >= 360.0) target_deg -= 360.0;

  while (pros::millis() < deadline) {
    const double cur = xdrive::heading_deg();        // 0..360
    const double err = wrap180(target_deg - cur);    // -180..180

    if (std::fabs(err) <= kHeadingEpsDeg) break;

    // P control -> rotation command
    const double cmd = std::clamp(kP * err, -127.0, 127.0);
    int rot = (int)std::lround(cmd);

    // Cap by maxVel
    rot = std::clamp(rot, -maxVel, maxVel);

    xdrive::drive(0, 0, rot, false);

    // Keep auto-sort alive while turning
    //mech::update();

    pros::delay(kLoopDtMs);
  }

  xdrive::drive(0, 0, 0, false);

  // One last tick after stopping
  //mech::update();
}

// Relative IMU turn
void turn_by_deg(double delta_deg, int maxVel, double kP) {
  double target = xdrive::heading_deg() + delta_deg;
  while (target < 0) target += 360.0;
  while (target >= 360.0) target -= 360.0;
  turn_to_deg(target, maxVel, kP);
}

// Coarse wheel-degree based spin
void turn_by_wheel_deg(double wheel_deg, int vel) {
  xdrive::turn_cw_deg(wheel_deg, vel);
  //mech::update();
}

// ---------------- Utility ----------------
void wait_ms(uint32_t ms) {
  const uint32_t t0 = pros::millis();
  while (pros::millis() - t0 < ms) {
    //mech::update();        // keep auto-sort alive in auton
    pros::delay(10);
  }
}


// ---------------- Subsystems (wired to mech) ----------------
void intake_on(int pct) {
  mech::intake_set(pct);
}

void intake_off() {
  mech::intake_stop();
}

void intake_reverse(int pct) {
  mech::intake_set(-std::abs(pct));
}

void feeder_intake_store() {
  // Auto-sort will run automatically during IntakeStore mode (mech::update()).
  mech::set_feeder_mode(feeder::Mode::IntakeStore);
}

void feeder_off() {
  mech::feeder_stop();
}

void feeder_score_lower() {
  mech::set_feeder_mode(feeder::Mode::ScoreLower);
}

void feeder_score_middle() {
  mech::set_feeder_mode(feeder::Mode::ScoreMiddle);
}

void feeder_score_upper() {
  mech::set_feeder_mode(feeder::Mode::ScoreUpper);
}

void loader_deploy()  { 
  mech::loader_deploy(); 
}

void loader_retract() { 
  mech::loader_retract(); 
}

} // namespace auton_actions
