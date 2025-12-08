#include "odom_sys.hpp"
#include "config/drive_select.hpp"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include <cmath>
#include <iostream>
#include <iomanip>
#include "pros/rtos.hpp"
#include <atomic>
#include <cstdio> 

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
  printf("[ODOM] init_once_for_auton: g_odom created\n"); fflush(stdout);

  // Optionally flip a sensor if mounted “backwards”
  // g_trackingwheel_par.set_reversed(true);
  // g_trackingwheel_perp.set_reversed(true);

  // Initialize last readings
  last_par_deg  = g_trackingwheel_par.get_position();   // degrees (cumulative)
  last_perp_deg = g_trackingwheel_perp.get_position();

  // start terminal printing
    odom_sys::start_debug();
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

    // ------------- Robust heading handling ---------------
      // Try rotation first ([-180,180]); if invalid, try heading ([0,360)).
    double h_deg = g_imu.get_rotation();
    if (!std::isfinite(h_deg)) h_deg = g_imu.get_heading();
    // Cache the last known-good heading in radians.
    static double last_good_h_rad = 0.0;
    static bool   have_good_h     = true;
    // Read IMU heading (degrees)
    double h_rad;
    if (std::isfinite(h_deg)) {
      h_rad = h_deg * (M_PI/180.0);
      last_good_h_rad = h_rad;
      have_good_h = true;
    } else {
      // IMU still invalid. If we have a last-good heading, use it; otherwise, skip this update.
      if (have_good_h) {
        h_rad = last_good_h_rad;
      } else {
        // First frames before IMU is sane: don't contaminate odom with NaNs.
        return;
      }
    }
      
    //---------------------------------------

    // Update odometry math
    g_odom->update(dPar_in, dPerp_in, h_rad);
}

// Get the latest pose estimate
Pose odom_sys::pose(){ 
    return g_odom->pose(); 
}

// ===================== Debug Task Implementation =====================

namespace {
  // runtime toggle (off by default, flip true when you want prints)
  std::atomic<bool> g_debug_enabled{true};

  void odom_debug_loop() {
    while (true) {
      if (g_debug_enabled.load(std::memory_order_relaxed) && g_odom) {
        Pose p = g_odom->pose();
        std::cout << std::fixed << std::setprecision(2)
                  << "[Odom]  X: " << p.x
                  << "  Y: " << p.y
                  << "  Th: " << (p.theta * 180.0 / M_PI) << "°"
                  << std::endl;
      }
      pros::delay(100); // 10 Hz
    }
  }
}

// Start the background terminal-print task (idempotent)
void odom_sys::start_debug() {
  static bool started = false;
  if (!started) {
    started = true;
    static pros::Task s_debug_task([] { odom_debug_loop(); });
    printf("[OdomDbg] task started\n"); fflush(stdout);
  }
}

// Toggle printing at runtime
void odom_sys::set_debug_enabled(bool en) {
  g_debug_enabled.store(en, std::memory_order_relaxed);
}

// ============== debuggin printout =============
void odom_sys::start_updater_task() {
  static bool started = false;
  if (!started) {
    started = true;
    static pros::Task s_updater([] {
      while (true) { update_from_hardware(); pros::delay(10); }
    });
  }
}

// ============== raw data logger =============
void odom_sys::start_raw_logger_task() {
  static bool started = false;
  if (started) return;
  started = true;

  static pros::Task s_raw([] {
    printf("[RAW] logger started. circ=%.3f in  Lpar=%.3f  Lperp=%.3f\n",
           TRACKING_WHEEL_DIAMETER_IN * M_PI,
           TRACKING_WHEEL_PARALLEL_OFFSET_IN,
           TRACKING_WHEEL_PERP_OFFSET_IN);
    fflush(stdout);

    double last_par = 0, last_perp = 0;
    while (true) {
      const double par = g_trackingwheel_par.get_position();   // deg
      const double perp = g_trackingwheel_perp.get_position(); // deg
      const double dPar = par - last_par;   last_par = par;
      const double dPerp = perp - last_perp; last_perp = perp;

      const double dPar_in  = (dPar  / 360.0) * (TRACKING_WHEEL_DIAMETER_IN * M_PI);
      const double dPerp_in = (dPerp / 360.0) * (TRACKING_WHEEL_DIAMETER_IN * M_PI);
      const double heading_deg = g_imu.get_rotation();

      printf("[RAW] par=%.1f° dPar=%.1f° (%.3f in) | perp=%.1f° dPerp=%.1f° (%.3f in) | imu=%.2f°\n",
             par, dPar, dPar_in, perp, dPerp, dPerp_in, heading_deg);
      fflush(stdout);

      pros::delay(250);
    }
  });
}