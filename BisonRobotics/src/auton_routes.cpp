#include "autons/auton_routes.hpp"
#include "autons/auton_actions.hpp"
#include "mech.hpp"                
#include "config/drive_select.hpp" 
#include <cmath>

// NOTE: Keep routes “high level”.
// Routes should only call auton_actions::* and avoid direct motor access.

// Helper: inches -> wheel degrees (geometry only, not odometry)
static inline double inches_to_wheel_deg(double inches, double wheel_diam_in) {
  return 360.0 * (inches / (M_PI * wheel_diam_in));
}

namespace auton_routes {

void run(AutonRoutine which) {
  using namespace auton_actions;

    // ================= Tunables for this family of routines ==================
    constexpr double FIRST_LEG_IN   = 36.0;  // forward (3 ft)
    constexpr double SECOND_LEG_IN  = 12.0;  // forward (1 ft)
    constexpr double TURN_DEG       = 90.0;  // turn angle
    constexpr int    DRIVE_VEL1     = 110;   // first leg speed
    constexpr int    DRIVE_VEL2     = 90;    // second leg speed
    constexpr int    TURN_MAXVEL    = 80;    // turn max velocity
  // ========================================================================

  switch (which) {

    case AutonRoutine::Red_Left: {
      mech::set_team_color(TeamColor::Red);

      intake_on(127);
      feeder_intake_store();

      //15
      turn_by_deg(-30, TURN_MAXVEL);   // left (CCW)
      wait_ms(100);

      drive_inches(45, DRIVE_VEL1);
      wait_ms(150);

      //75
      turn_by_deg(-150, TURN_MAXVEL);   // left (CCW)
      wait_ms(100);

      drive_inches(24, DRIVE_VEL1);
      wait_ms(100);

      //90
      turn_by_deg(-180, TURN_MAXVEL);   // left (CCW)
      wait_ms(100);

      drive_inches(55, DRIVE_VEL1);
      wait_ms(100);

      strafe_left_inches(13, DRIVE_VEL1);
      wait_ms(100);

      loader_deploy();
      wait_ms(500);

      drive_inches(20, DRIVE_VEL1);
      wait_ms(3000); // wait to ensure loading

      drive_inches(-20, DRIVE_VEL1); // back away
      wait_ms(100);

      loader_retract();
      wait_ms(500);

      //180
      turn_by_deg(360, TURN_MAXVEL);   // turn around
      wait_ms(100);

      drive_inches(20, DRIVE_VEL1);
      wait_ms(100);

      feeder_score_upper();
      wait_ms(3000);  // wait to ensure scoring

      feeder_intake_store();
      wait_ms(100);

      drive_inches(-20, DRIVE_VEL1); // back away
      wait_ms(100);

      //45
      turn_by_deg(+90, TURN_MAXVEL);   // right (CW)
      wait_ms(100);

      drive_inches(55, DRIVE_VEL1);
      wait_ms(100);

      feeder_score_middle();
      wait_ms(3000);  // wait to ensure scoring

      park_brake();
      intake_off();
      feeder_off();
    } break;

    case AutonRoutine::Red_Right: {
      mech::set_team_color(TeamColor::Red);

      intake_on(127);
      feeder_intake_store();

      drive_inches(FIRST_LEG_IN, DRIVE_VEL1);
      wait_ms(150);

      turn_by_deg(+TURN_DEG, TURN_MAXVEL);   // right (CW)
      wait_ms(100);

      drive_inches(SECOND_LEG_IN, DRIVE_VEL2);

      intake_off();
      feeder_off();
    } break;

    case AutonRoutine::Blue_Left: {
      mech::set_team_color(TeamColor::Blue);

      intake_on(127);
      feeder_intake_store();

      drive_inches(FIRST_LEG_IN, DRIVE_VEL1);
      wait_ms(150);

      turn_by_deg(-TURN_DEG, TURN_MAXVEL);   // left (CCW)
      wait_ms(100);

      drive_inches(SECOND_LEG_IN, DRIVE_VEL2);

      intake_off();
      feeder_off();
    } break;

    case AutonRoutine::Blue_Right: {
      mech::set_team_color(TeamColor::Blue);

      intake_on(127);
      feeder_intake_store();

      drive_inches(FIRST_LEG_IN, DRIVE_VEL1);
      wait_ms(150);

      turn_by_deg(+TURN_DEG, TURN_MAXVEL);   // right (CW)
      wait_ms(100);

      drive_inches(SECOND_LEG_IN, DRIVE_VEL2);

      intake_off();
      feeder_off();
    } break;

    // 60 sec skills auton
    case AutonRoutine::Skills: {
      intake_on(127);
      feeder_intake_store();

      // TODO: your skills routine

      intake_off();
      feeder_off();
    } break;

    default: {
      // Safe default
      intake_off();
      feeder_off();
      wait_ms(250);
    } break;
  }
}

} // namespace auton_routes
