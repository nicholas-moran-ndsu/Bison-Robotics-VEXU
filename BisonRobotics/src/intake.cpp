#include "subsystems/intake.hpp"
#include "mech.hpp"
#include "config/drive_select.hpp"
#include "pros/apix.h"

namespace {
  // Port-only ctor (most compatible)
  pros::Motor mIntake(PORT_INTAKE);
}

void intake::initialize() {
  // Configure once at startup
  mIntake.set_gearing(pros::E_MOTOR_GEARSET_18);           //Green
  mIntake.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
  mIntake.set_reversed(INTAKE_REVERSED);
  mIntake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

void intake::set(int pct) { mIntake.move(pct); }
void intake::stop()       { mIntake.move(0);   }
