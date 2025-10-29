#include "odom_sys.hpp"
#include "config/drive_select.hpp"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include <cmath>

// ---- Hardware objects ----
static pros::Imu g_imu(IMU_PORT);
static pros::Rotation g_trackingwheel_par(TRACKING_WHEEL_PARALLEL_PORT);
static pros::Rotation g_trackingwheel_perp(TRACKING_WHEEL_PERPENDICULAR_PORT);

// ---- Helper functions ----
static constexpr double kCirc = TRACKING_WHEEL_DIAMETER_IN * M_PI;
static inline double deg_to_in(double deg){ return (deg / 360.0) * kCirc; }

// ---- Odometry system ----
static Odom2WIMU* g_odom = nullptr;
static double last_par_deg = 0.0, last_perp_deg = 0.0;

// Initialize encoders and IMU (run once at start of auton)
void odom_sys::init_once_for_auton(){
  // Calibrate IMU only for auton
  g_imu.reset();
  while (g_imu.is_calibrating()) pros::delay(10);

  // Zero rotation sensors
  g_trackingwheel_par.reset_position();
  g_trackingwheel_perp.reset_position();

  // Create odometry object
  OdomConfig cfg{
    TRACKING_WHEEL_PARALLEL_OFFSET_IN,
    TRACKING_WHEEL_PERP_OFFSET_IN,
    {0,0,0}
  };

  static Odom2WIMU odom(cfg);
  g_odom = &odom;

  // Optionally flip a sensor if mounted “backwards”
  // g_trackingwheel_par.set_reversed(true);
  // g_trackingwheel_perp.set_reversed(true);

  // Initialize last readings
  last_par_deg  = g_trackingwheel_par.get_position();   // degrees (cumulative)
  last_perp_deg = g_trackingwheel_perp.get_position();
}

// Set robot pose (e.g., start tile alignment)
void odom_sys::reset_pose(const Pose& p){
    // Reset odometry math
    g_odom->reset(p);
    // Reset hardware encoders
    g_trackingwheel_par.reset_position();
    g_trackingwheel_perp.reset_position();
    // Initialize last readings
    last_par_deg  = g_trackingwheel_par.get_position();
    last_perp_deg = g_trackingwheel_perp.get_position();
}

// Read sensors and update the odometry math
void odom_sys::update_from_hardware(){
    // Read current encoder positions
    const double par_deg  = g_trackingwheel_par.get_position();
    const double perp_deg = g_trackingwheel_perp.get_position();
    // Compute deltas since last update
    const double dPar_deg  = par_deg  - last_par_deg;   last_par_deg  = par_deg;
    const double dPerp_deg = perp_deg - last_perp_deg;  last_perp_deg = perp_deg;
    // Convert to inches
    const double dPar_in  = deg_to_in(dPar_deg);
    const double dPerp_in = deg_to_in(dPerp_deg);
    // Read heading from IMU (convert to radians)
    const double heading  = g_imu.get_rotation() * (M_PI/180.0);
    // Update odometry math
    g_odom->update(dPar_in, dPerp_in, heading);
}

// Get the latest pose estimate
Pose odom_sys::pose(){ 
    return g_odom->pose(); 
}
