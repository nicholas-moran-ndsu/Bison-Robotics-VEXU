#include "subsystems/feeder.hpp"
#include "subsystems/config.hpp"

namespace {
  pros::Motor mFeeder(PORT_FEEDER);  // <-- port only
}

void feeder::initialize() {
  mFeeder.set_gearing(pros::E_MOTOR_GEARSET_18);        // or your constant
  mFeeder.set_reversed(FEEDER_REVERSED);
  mFeeder.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
  mFeeder.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void feeder::set(int pct) { mFeeder.move(pct); }
void feeder::stop()       { mFeeder.move(0);   }

void feeder::run_for_ms(int p, uint32_t ms) {
  mFeeder.move(p);
  pros::delay(ms);
  mFeeder.move(0);
}
