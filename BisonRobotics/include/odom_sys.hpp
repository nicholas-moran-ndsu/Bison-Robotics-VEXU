#pragma once
#include "odom.hpp"

namespace odom_sys {
  // Initialize encoders and IMU (run once at start of auton)
  void init_once_for_auton();

  // Set robot pose (e.g., start tile alignment)
  void reset_pose(const Pose& p);

  // Read sensors and update the odometry math
  void update_from_hardware();

  // Get the latest pose estimate
  Pose pose();

  // spawn the cout debug task (10 Hz)
  void start_debug();    

  // optional runtime toggle  
  void set_debug_enabled(bool); 

  // runs update_from_hardware() at ~100 Hz
  void start_updater_task(); 

  // optional: start a raw data logger task
  void start_raw_logger_task();

} // namespace odom_sys