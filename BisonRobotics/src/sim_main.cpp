#ifdef SIM
#include <cstdio>
#include <vector>
#include <cmath>
#include "xdrive.hpp"
#include "odom.hpp"
#include "sim_compat.hpp"

using xdrive::drive;

// Convert 4 wheel commands (what your code sends) back into chassis commands.
// Your mapping is:
//   fl = df + ds + dr
//   fr = df - ds - dr
//   bl = df - ds + dr
//   br = df + ds - dr
// So invert:
//   df = (fl+fr+bl+br)/4
//   ds = (fl - fr - bl + br)/4
//   dr = (fl - fr + bl - br)/4
struct WheelCmd { double fl, fr, bl, br; };

// Read the last commanded values from the mocks
static WheelCmd read_wheels() {
  // accesses the internal mocks via the voltage trick (proportional)
  // but we don't have direct globals here; instead, we’ll call drive() and
  // immediately infer from the arguments we used. To keep it simple, we
  // just store/compute the df,ds,dr directly in the pattern below.
  return {0,0,0,0}; // placeholder (we compute from df,ds,dr where we call drive)
}

struct Cmd { double t_s; int fwd, str, rot; bool field; };

int main() {
  // ---- Initialize (no hardware) ----
  xdrive::initialize();

  // ---- Odometry model (2 wheels + IMU) ----
  OdomConfig cfg; cfg.L_par=3.0; cfg.L_perp=4.0; cfg.start={0,0,0};
  Odom2WIMU odom(cfg);

  // ---- Simple command script (joystick space) ----
  std::vector<Cmd> plan = {
    {2.0, +90,   0,   0, false},  // forward
    {1.0,   0, +90,   0, false},  // right
    {1.5,   0,   0, +90, false},  // rotate CW (positive in your mapping)
    {1.0, +64, +64,   0, false},  // diagonal
  };

  // Scaling joystick to physical motion (tune these to your robot feel)
  const double max_v_ips   = 30.0;     // “full stick forward” inches/sec
  const double max_w_rps   = M_PI/1.0; // “full stick rot” rad/sec  (180°/s)
  const double dt          = 0.01;

  // Ground truth pose (what we integrate from joystick intent)
  Pose gt{0,0,0};

  std::puts("time_s, gt_x, gt_y, gt_th, est_x, est_y, est_th, df, ds, dr");

  double t=0.0;
  for (auto c: plan) {
    const int steps = (int)std::round(c.t_s/dt);
    for (int k=0; k<steps; ++k) {
      // ---- Call your drive() just like teleop would ----
      drive(c.fwd, c.str, c.rot, c.field);

      // Recreate df,ds,dr exactly as your code does (deadband + square)
      auto db = [](int v){ return (std::abs(v) < xdrive::DEADBAND) ? 0 : v; };
      auto sq = [](int v){ double s=v/127.0; return std::copysign(s*s,s)*127.0; };
      const int f = db(c.fwd), s = db(c.str), r = db(c.rot);
      const double df = xdrive::SQUARE_INPUTS ? sq(f) : f;
      const double ds = xdrive::SQUARE_INPUTS ? sq(s) : s;
      const double dr = xdrive::SQUARE_INPUTS ? sq(r) : r;

      // Map joystick-space to physical velocities
      const double vy_r = (df/127.0) * max_v_ips;  // +forward
      const double vx_r = (ds/127.0) * max_v_ips;  // +right
      const double w    = (dr/127.0) * max_w_rps;  // +CCW (note: your +rot is CW; adjust sign)
      // Your mapping labeled +dr as CW; odom assumes +theta is CCW.
      // Flip sign for physics:
      const double omega = -w;

      // Integrate GT in field frame (midpoint)
      const double thm = gt.theta + 0.5*omega*dt;
      const double cth = std::cos(thm), sth = std::sin(thm);
      gt.x += ( cth*vx_r - sth*vy_r) * dt;
      gt.y += ( sth*vx_r + cth*vy_r) * dt;
      gt.theta = Odom2WIMU::wrap(gt.theta + omega*dt);

      // Tracking-wheel deltas from robot-centric dx,dy and dtheta
      const double dx_r = vx_r*dt, dy_r = vy_r*dt, dth = omega*dt;
      const double sPar  = dy_r - cfg.L_par  * dth;
      const double sPerp = dx_r + cfg.L_perp * dth;

      // IMU heading is absolute field orientation
      const double imu_heading_rad = gt.theta;

      // Feed odometry
      odom.update(sPar, sPerp, imu_heading_rad);

      // Log
      const Pose est = odom.pose();
      std::printf("%.3f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.2f, %.2f, %.2f\n",
                  t, gt.x, gt.y, gt.theta, est.x, est.y, est.theta, df, ds, dr);

      t += dt;
      sleep_ms((uint32_t)(dt*1000));
    }
  }

  return 0;
}
#endif