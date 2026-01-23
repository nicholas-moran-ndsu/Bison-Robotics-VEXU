#include "subsystems/feeder.hpp"
#include "mech.hpp"
#include "config/drive_select.hpp"
#include "pros/apix.h"

namespace {
  // NOTE: set these PORT_* macros in config.hpp
  // 1/3 are “tied together” → one motor controls both
  pros::Motor mFeed13(PORT_FEEDER_1_3);  // does positions 1 & 3
  pros::Motor mFeed2 (PORT_FEEDER_2);
  pros::Motor mFeed4 (PORT_FEEDER_4);
  pros::Motor mFeed5 (PORT_FEEDER_5);
  pros::Motor mFeed6 (PORT_FEEDER_6);

  constexpr int kFeederSpeed = 127; // full send. tweak if needed

  // --- override state for motor2 ---
  bool o2_en = false;
  int  o2_pct = 0;
}

void feeder::initialize() {
  // Green - 200 RPM
  mFeed13.set_gearing(pros::E_MOTOR_GEARSET_18);
  mFeed2.set_gearing(pros::E_MOTOR_GEARSET_18);
  mFeed4.set_gearing(pros::E_MOTOR_GEARSET_18);
  mFeed5.set_gearing(pros::E_MOTOR_GEARSET_18);
  mFeed6.set_gearing(pros::E_MOTOR_GEARSET_18);

  mFeed13.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
  mFeed2.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
  mFeed4.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
  mFeed5.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
  mFeed6.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

  // Set reversals from xdrive8.hpp
  mFeed13.set_reversed(REVERSED_F_1_3);
  mFeed2.set_reversed(REVERSED_F_2);
  mFeed4.set_reversed(REVERSED_F_4);
  mFeed5.set_reversed(REVERSED_F_5);
  mFeed6.set_reversed(REVERSED_F_6);

  mFeed13.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  mFeed2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  mFeed4.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  mFeed5.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  mFeed6.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

void feeder::set_mode(Mode m) {
  int s13 = 0, s2 = 0, s4 = 0, s5 = 0, s6 = 0;

  switch (m) {
    case Mode::Off:
      // all 0 already
      break;

    case Mode::IntakeStore:
      // 1/3 CCW, 2 CW, 4 CCW, 5 CCW, 6 CW
      s13 = +kFeederSpeed;
      s2  = -kFeederSpeed;
      s4  = +kFeederSpeed;
      s5  = +kFeederSpeed;
      s6  = -kFeederSpeed;
      break;

    case Mode::ScoreLower:
      // 1/3 CW, 2 CCW, 4 CW, 5 CW, 6 CCW
      s13 = -kFeederSpeed;
      s2  = +kFeederSpeed;
      s4  = -kFeederSpeed;
      s5  = -kFeederSpeed;
      s6  = +kFeederSpeed;
      break;

    case Mode::ScoreMiddle:
      // 1/3 CCW, 2 CCW, 4 CW, 5 CW, 6 CW
      s13 = +kFeederSpeed;
      s2  = +kFeederSpeed;
      s4  = -kFeederSpeed;
      s5  = -kFeederSpeed;
      s6  = -kFeederSpeed;
      break;

    case Mode::ScoreUpper:
      // 1/3 CCW, 2 CCW, 4 CW, 5 CCW, 6 CW
      s13 = +kFeederSpeed;
      s2  = +kFeederSpeed;
      s4  = -kFeederSpeed;
      s5  = +kFeederSpeed;
      s6  = -kFeederSpeed;
      break;
  }

  // --- override wins over mode output ---
  if (o2_en) s2 = o2_pct;

  mFeed13.move(s13);
  mFeed2 .move(s2);
  mFeed4 .move(s4);
  mFeed5 .move(s5);
  mFeed6 .move(s6);
}

// --- override functions ---
void feeder::override_motor2(int pct) {
  o2_en = true;
  o2_pct = pct;
}
void feeder::clear_overrides() {
  o2_en = false;
  o2_pct = 0;
}

void feeder::stop() {
  clear_overrides();
  set_mode(Mode::Off);
}