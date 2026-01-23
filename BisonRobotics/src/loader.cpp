#include "subsystems/loader.hpp"
#include "config/drive_select.hpp"
#include "mech.hpp"

using namespace pros;

namespace loader {

  // Digital output controlling the solenoid valve
  static pros::adi::DigitalOut g_sol(LOADER_SOLENOID_PORT);

  static bool g_deployed = false;

  // Apply current state to hardware (handles inversion)
  static void apply() {
    const bool out = LOADER_SOLENOID_INVERTED ? !g_deployed : g_deployed;
    g_sol.set_value(out);
  }

  void initialize() {
    // Start safe (usually retracted)
    g_deployed = false;
    apply();
  }

  void set_deployed(bool deployed) {
    g_deployed = deployed;
    apply();
  }

  void deploy()  { set_deployed(true);  }
  void retract() { set_deployed(false); }

  bool is_deployed() { return g_deployed; }

} // namespace loader
