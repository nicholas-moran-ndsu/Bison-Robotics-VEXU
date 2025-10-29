#pragma once
#include <cmath>
// Represents the robot’s position and orientation in global (field) coordinates
struct Pose { double x, y, theta; }; // inches, inches, radians

// Defines robot geometry and starting conditions
struct OdomConfig {
  double L_par;   // +forward wheel offset (inches)
  double L_perp;  // +right   wheel offset (inches)
  Pose   start{0,0,0};
};

// 2-Wheel Odometry with IMU class
class Odom2WIMU {
 public:
  explicit Odom2WIMU(const OdomConfig& c)
    : cfg(c), p(c.start), last_h(c.start.theta) {}
  
  // Reset to starting pose or specified pose
  void reset(const Pose& start = {0,0,0}) {
    p = start;
    last_h = start.theta;
  }

  void set_pose(const Pose& s){ 
    p = s; 
    last_h = s.theta; 
  }

  // Update pose given new wheel displacements and heading
  void update(double sPar_in, double sPerp_in, double heading_rad) {
    const double dth = wrap(heading_rad - last_h);    // heading change
    last_h = heading_rad;                             // update last heading
    // Compute deltas in robot-centric frame
    const double dx_r =  sPerp_in - cfg.L_perp * dth; // +right
    const double dy_r =  sPar_in  + cfg.L_par  * dth; // +forward
    // Compute average heading during the motion
    const double thm  = p.theta + 0.5*dth;
    // Rotate deltas to field-relative and update pose
    const double c = std::cos(thm), s = std::sin(thm);
    p.x +=  c*dx_r - s*dy_r;
    p.y +=  s*dx_r + c*dy_r;
    p.theta = wrap(p.theta + dth);
  }
  
  // Get current position
  Pose pose() const { return p; }
  
  // Wrap angle to (-pi, pi]
  static double wrap(double a){ 
    while(a> M_PI)a-=2*M_PI; 
    while(a<=-M_PI)a+=2*M_PI; 
    return a; 
  }
  
private:
  // Internal state
  OdomConfig cfg; 
  Pose p; 
  double last_h;
};