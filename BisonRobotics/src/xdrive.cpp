#include "xdrive.hpp"
#include "config/drive_select.hpp"   // sets XDRIVE_CORNER_MOTORS and ports/reversals
#include <cmath>
#include <array>
#include <algorithm>

//============= testing =============

// --- Port range guards (V5 smart ports are 1..21) ---
#define _CHK_PORT(P) static_assert((P) >= 1 && (P) <= 21, "Bad motor port: " #P)
_CHK_PORT(PORT_FL1);
_CHK_PORT(PORT_FR1);
_CHK_PORT(PORT_BL1);
_CHK_PORT(PORT_BR1);
#if XDRIVE_CORNER_MOTORS == 3
_CHK_PORT(PORT_FL2); _CHK_PORT(PORT_FL3);
_CHK_PORT(PORT_FR2); _CHK_PORT(PORT_FR3);
_CHK_PORT(PORT_BL2); _CHK_PORT(PORT_BL3);
_CHK_PORT(PORT_BR2); _CHK_PORT(PORT_BR3);
#endif
#undef _CHK_PORT

//============= end testing =============

namespace xdrive {

// ================= Config overview =================
// XDRIVE_CORNER_MOTORS == 1  → 4 total motors (1 per corner)
// XDRIVE_CORNER_MOTORS == 3  → 12 total motors (3 per corner)
static_assert(XDRIVE_CORNER_MOTORS == 1 || XDRIVE_CORNER_MOTORS == 3,
              "XDRIVE_CORNER_MOTORS must be 1 or 3");

// ---------------- MotorGroup helper ----------------
template <typename MotorT, size_t N>
struct MotorGroup {
  std::array<MotorT*, N> m;

  template <class F>
  inline void each(F&& f) { for (auto* p : m) f(*p); }

  // Config
  void set_gearing(pros::motor_gearset_e_t g) { each([&](MotorT& x){ x.set_gearing(g); }); }
  void set_encoder_units(pros::motor_encoder_units_e_t e) { each([&](MotorT& x){ x.set_encoder_units(e); }); }
  void set_reversed(bool r) { each([&](MotorT& x){ x.set_reversed(r); }); }

  // Commands
  void move(int v) { each([&](MotorT& x){ x.move(v); }); }
  void move_relative(double deg, int speed) { each([&](MotorT& x){ x.move_relative(deg, speed); }); }
  void tare_position() { each([&](MotorT& x){ x.tare_position(); }); }

  // Telemetry
  double get_voltage() const {
    double s = 0; for (auto* p : m) s += p->get_voltage(); return s / double(N);
  }
  double get_actual_velocity() const {
    double s = 0; for (auto* p : m) s += p->get_actual_velocity(); return s / double(N);
  }
  double get_position_abs_max() const {
    double mx = 0; for (auto* p : m) mx = std::max(mx, std::abs(p->get_position())); return mx;
  }
  double get_position() const {
    double s = 0; for (auto* p : m) s += p->get_position(); return s / double(N);
  }
};

// --------------- Motors & IMU (PROS) ---------------
#if XDRIVE_CORNER_MOTORS == 3
  static pros::Motor mFL1(PORT_FL1), mFL2(PORT_FL2), mFL3(PORT_FL3);
  static pros::Motor mFR1(PORT_FR1), mFR2(PORT_FR2), mFR3(PORT_FR3);
  static pros::Motor mBL1(PORT_BL1), mBL2(PORT_BL2), mBL3(PORT_BL3);
  static pros::Motor mBR1(PORT_BR1), mBR2(PORT_BR2), mBR3(PORT_BR3);
#else
  static pros::Motor mFL1(PORT_FL1);
  static pros::Motor mFR1(PORT_FR1);
  static pros::Motor mBL1(PORT_BL1);
  static pros::Motor mBR1(PORT_BR1);
#endif

static pros::Imu imu(IMU_PORT);

using Corner = MotorGroup<pros::Motor, XDRIVE_CORNER_MOTORS>;

#if XDRIVE_CORNER_MOTORS == 3
  static Corner gFL{{ &mFL1, &mFL2, &mFL3 }};
  static Corner gFR{{ &mFR1, &mFR2, &mFR3 }};
  static Corner gBL{{ &mBL1, &mBL2, &mBL3 }};
  static Corner gBR{{ &mBR1, &mBR2, &mBR3 }};
#else
  static Corner gFL{{ &mFL1 }};
  static Corner gFR{{ &mFR1 }};
  static Corner gBL{{ &mBL1 }};
  static Corner gBR{{ &mBR1 }};
#endif

#if IMU_PORT >= 0 // only compile IMU-related code if IMU_PORT is valid

// Wraps angle to (-180, 180] instead of [0, 360)
static inline double wrap180(double d) { 
  // wrap to (-180, 180]
  while (d <= -180.0) d += 360.0;
  while (d >   180.0) d -= 360.0;
  return d;
}
#endif

//deadband: if within deadband, return 0; else return original value
static inline int deadband(int v) {
  return (std::abs(v) < DEADBAND) ? 0 : v;
}

// Apply exponential scaling to joystick input for finer low-speed control
static inline double expo_scale(int v) {
  const double s = v / 127.0;                                   // scale to -1.0..1.0     
  const double mag = std::pow(std::abs(s), DRIVE_EXPONENT);     //sets magnitude to abs value ^ exponent
  return std::copysign(mag, s) * 127.0;                         // restore sign and scale back to -127..127
}

// Initialize motors + IMU (call once at startup)
void initialize() {
  // Gearing & encoders on every motor in each corner
  gFL.set_gearing(GEARSET); gFR.set_gearing(GEARSET);
  gBL.set_gearing(GEARSET); gBR.set_gearing(GEARSET);

  gFL.set_encoder_units(ENCODERS); gFR.set_encoder_units(ENCODERS);
  gBL.set_encoder_units(ENCODERS); gBR.set_encoder_units(ENCODERS);

  // Per-motor reversals
#if XDRIVE_CORNER_MOTORS == 3
  mFL1.set_reversed(REVERSED_FL1); mFL2.set_reversed(REVERSED_FL2); mFL3.set_reversed(REVERSED_FL3);
  mFR1.set_reversed(REVERSED_FR1); mFR2.set_reversed(REVERSED_FR2); mFR3.set_reversed(REVERSED_FR3);
  mBL1.set_reversed(REVERSED_BL1); mBL2.set_reversed(REVERSED_BL2); mBL3.set_reversed(REVERSED_BL3);
  mBR1.set_reversed(REVERSED_BR1); mBR2.set_reversed(REVERSED_BR2); mBR3.set_reversed(REVERSED_BR3);
#else
  mFL1.set_reversed(REVERSED_FL1);
  mFR1.set_reversed(REVERSED_FR1);
  mBL1.set_reversed(REVERSED_BL1);
  mBR1.set_reversed(REVERSED_BR1);
#endif

  // IMU calibration (zeroing gyro/accelerometer)
  zero_field_forward();
}

double heading_deg() {
#if IMU_PORT >= 0
  // Use the SAME API everywhere (get_heading is 0..360)
  return imu.get_heading();
#else
  return 0.0;
#endif
}

// IMU calibration (zeroing gyro/accelerometer)
void zero_field_forward() {
  if (IMU_PORT > 0) {
    imu.reset();   // Begin calibration
    // Wait up to ~2.5 seconds (250 × 10 ms) while the IMU calibrates
    // During this time, the IMU measures bias and stabilizes its sensors
    for (int t = 0; t < 250 && imu.is_calibrating(); ++t)
      pros::delay(10);  // Small delay to avoid blocking the CPU
  }
}

// Normalize 4 wheel values to [-127..127]
static void normalize(double &fl, double &fr, double &bl, double &br) {
  const double maxmag = std::max({std::abs(fl), std::abs(fr), std::abs(bl), std::abs(br), 127.0});
  if (maxmag > 127.0) {
    const double k = 127.0 / maxmag;
    fl *= k; fr *= k; bl *= k; br *= k;
  }
}

// Main drive function
void drive(int fwd, int str, int rot, bool field_centric) {
  fwd = deadband(fwd);  // forward
  str = deadband(str);  // strafe
  rot = deadband(rot);  // rotate

  double df = expo_scale(fwd); // forward
  double ds = expo_scale(str); // strafe
  double dr = expo_scale(rot); // rotate

  // Apply field-centric transform
  #ifndef SIM
    if (field_centric && IMU_PORT > 0 && !imu.is_calibrating()) {
      const double th = -heading_deg() * (M_PI / 180.0);
      const double c = std::cos(th), s = std::sin(th);
      const double rs =  ds * c + df * s;   // strafe
      const double rf =  df * c - ds * s;   // forward
      ds = rs; df = rf;
    }
  #else
    // In SIM, we always allow field_centric off (IMU mock is absolute anyway).
    if (field_centric) {
      const double th = heading_deg() * (M_PI / 180.0);
      const double c = std::cos(th), s = std::sin(th);
      const double rs =  ds * c + df * s;
      const double rf =  df * c - ds * s;
      ds = rs; df = rf;
    }
  #endif

  // robot centric X-drive kinematics: +df=forward, +ds=right, +dr=Clockwise
  double fl = df + ds + dr;
  double fr = df - ds - dr;
  double bl = df - ds + dr;
  double br = df + ds - dr;

  normalize(fl, fr, bl, br);

  gFL.move(static_cast<int>(fl));
  gFR.move(static_cast<int>(fr));
  gBL.move(static_cast<int>(bl));
  gBR.move(static_cast<int>(br));
}

// ---- Simple open-loop autonomous helpers ----

// Reset all motor positions to zero
static void reset_positions() {
  #ifndef SIM
  gFL.tare_position(); gFR.tare_position();
  gBL.tare_position(); gBR.tare_position();
  #endif
}

// Move all 4 wheels relative to current position
static void move_all_relative(double fl, double fr, double bl, double br, int speed) {
  #ifndef SIM
  gFL.move_relative(fl, speed);
  gFR.move_relative(fr, speed);
  gBL.move_relative(bl, speed);
  gBR.move_relative(br, speed);
  #endif
}

// Return true if any motor is still busy moving to target (within tol)
static bool any_busy(double target_deg, double tol = 5.0) {
  #ifndef SIM
  const double T = std::max(0.0, std::abs(target_deg) - tol);
  return (std::abs(gFL.get_position()) < T) ||
         (std::abs(gFR.get_position()) < T) ||
         (std::abs(gBL.get_position()) < T) ||
         (std::abs(gBR.get_position()) < T);
  #else
  return false;
#endif
}

// Move in a given direction (blocking)
void drive_forward_deg(double wheel_deg, int speed) {
  reset_positions();
  move_all_relative(wheel_deg, wheel_deg, wheel_deg, wheel_deg, speed);
  #ifndef SIM
  pros::delay(10);
  while (any_busy(wheel_deg)) pros::delay(10);
  #endif
}

// strafe right is + on left wheels, - on right wheels
void strafe_right_deg(double wheel_deg, int speed) {
  reset_positions();
  move_all_relative(+wheel_deg, -wheel_deg, -wheel_deg, +wheel_deg, speed);
  #ifndef SIM
  pros::delay(10);
  while (any_busy(wheel_deg)) pros::delay(10);
  #endif
}

// turn clockwise is + on front left & back left, - on front right & back right
void turn_cw_deg(double wheel_deg, int speed) {
  reset_positions();
  move_all_relative(+wheel_deg, -wheel_deg, +wheel_deg, -wheel_deg, speed);
  #ifndef SIM
  pros::delay(10);
  while (any_busy(wheel_deg)) pros::delay(10);
  #endif
}

// ---------- LCD TELEMETRY ----------

/*
#ifndef SIM
static pros::Task* telemetry_task = nullptr;
#endif

// Telemetry task: periodically update the LCD with motor voltages + directions
#ifndef SIM
static void telemetry_loop(void*) {
  pros::lcd::initialize(); // safe to call if already initialized
  while (true) {
    // Read commanded voltage (mV). Sign indicates direction.
    const double vFL = gFL.get_voltage();
    const double vFR = gFR.get_voltage();
    const double vBL = gBL.get_voltage();
    const double vBR = gBR.get_voltage();

    // Convert to percent of full scale (~12000 mV on V5)
    auto pct = [](double mv) {
      const double p = (mv / 12000.0) * 100.0;
      // clamp for safety
      if (p > 100.0) return 100.0;
      if (p < -100.0) return -100.0;
      return p;
    };

    // Direction labels & magnitude
    const double pFL = pct(vFL), pFR = pct(vFR), pBL = pct(vBL), pBR = pct(vBR);
    auto dir = [](double p){ return p >= 0 ? "FWD" : "REV"; };

    // Optional: show actual velocity (RPM) to confirm motion
    const double rFL = gFL.get_actual_velocity();
    const double rFR = gFR.get_actual_velocity();
    const double rBL = gBL.get_actual_velocity();
    const double rBR = gBR.get_actual_velocity();

    // Print to LCD (rows 0–7)
    pros::lcd::print(0, "X-Drive Telemetry");
    pros::lcd::print(1, "FL: %4.0f%% %s | %4.0f rpm", fabs(pFL), dir(pFL), rFL);
    pros::lcd::print(2, "FR: %4.0f%% %s | %4.0f rpm", fabs(pFR), dir(pFR), rFR);
    pros::lcd::print(3, "BL: %4.0f%% %s | %4.0f rpm", fabs(pBL), dir(pBL), rBL);
    pros::lcd::print(4, "BR: %4.0f%% %s | %4.0f rpm", fabs(pBR), dir(pBR), rBR);

    // If you only want to show when powered, you could blank lines when |pct| < 1–2%.

    pros::delay(100); // update ~10 Hz
  }
}
#endif

// Start telemetry task (call once at startup)
void start_telemetry() {
  #ifndef SIM
  if (!telemetry_task) {
    telemetry_task = new pros::Task(telemetry_loop, nullptr, "xdrive-telemetry");
  }
  #endif
}

// Stop telemetry task (call once at shutdown)
void stop_telemetry() {
  #ifndef SIM
  if (telemetry_task) {
    telemetry_task->remove();
    delete telemetry_task;
    telemetry_task = nullptr;
  }
  #endif
}

*/

} // namespace xdrive