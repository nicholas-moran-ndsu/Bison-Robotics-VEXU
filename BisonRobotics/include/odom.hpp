#pragma once
#include <cmath>
struct Pose { double x, y, theta; }; // inches, inches, radians

struct OdomConfig {
  double L_par;   // +forward wheel offset (inches)
  double L_perp;  // +right   wheel offset (inches)
  Pose   start{0,0,0};
};

class Odom2WIMU {
 public:
  explicit Odom2WIMU(const OdomConfig& c): cfg(c), p(c.start), last_h(c.start.theta) {}
  void update(double sPar_in, double sPerp_in, double heading_rad) {
    const double dth = wrap(heading_rad - last_h); last_h = heading_rad;
    const double dx_r =  sPerp_in - cfg.L_perp * dth; // +right
    const double dy_r =  sPar_in  + cfg.L_par  * dth; // +forward
    const double thm  = p.theta + 0.5*dth;
    const double c = std::cos(thm), s = std::sin(thm);
    p.x +=  c*dx_r - s*dy_r;
    p.y +=  s*dx_r + c*dy_r;
    p.theta = wrap(p.theta + dth);
  }
  Pose pose() const { return p; }
  static double wrap(double a){ while(a> M_PI)a-=2*M_PI; while(a<=-M_PI)a+=2*M_PI; return a; }
 private:
  OdomConfig cfg; Pose p; double last_h;
};