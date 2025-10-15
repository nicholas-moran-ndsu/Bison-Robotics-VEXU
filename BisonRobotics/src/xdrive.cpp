#include "sim_compat.hpp"
#include "xdrive.hpp"
#include <cmath>

namespace xdrive {

#if IMU_PORT >= 0
static double field_zero_raw = 0.0;  // stores raw heading at the moment you press A

static inline double wrap180(double d) {
  // wrap to (-180, 180]
  while (d <= -180.0) d += 360.0;
  while (d >   180.0) d -= 360.0;
  return d;
}
#endif


// --- Construct motors with just the port, then set options via setters ---
#ifdef SIM
  // ---- SIM motors/IMU ----
  static MotorMock mFL(PORT_FL, REVERSED_FL);
  static MotorMock mFR(PORT_FR, REVERSED_FR);
  static MotorMock mBL(PORT_BL, REVERSED_BL);
  static MotorMock mBR(PORT_BR, REVERSED_BR);
  static ImuMock   imu;
#else  // ---- Real PROS motors/IMU ----
static pros::Motor mFL(PORT_FL);
static pros::Motor mFR(PORT_FR);
static pros::Motor mBL(PORT_BL);
static pros::Motor mBR(PORT_BR);
static pros::Imu   imu(IMU_PORT);
#endif

static inline int deadband(int v) {
  return (std::abs(v) < DEADBAND) ? 0 : v;
}

static inline double expo_scale(int v) {
  const double s = v / 127.0;
  const double mag = std::pow(std::abs(s), DRIVE_EXPONENT);
  return std::copysign(mag, s) * 127.0;
}

void initialize() {
#ifndef SIM  
  // Set each motor’s internal gear cartridge (e.g., green = 18:1)
  mFL.set_gearing(GEARSET);
  mFR.set_gearing(GEARSET);
  mBL.set_gearing(GEARSET);
  mBR.set_gearing(GEARSET);

  // Set encoder units for each motor (degrees, ticks, or rotations)
  mFL.set_encoder_units(ENCODERS);
  mFR.set_encoder_units(ENCODERS);
  mBL.set_encoder_units(ENCODERS);
  mBR.set_encoder_units(ENCODERS);

  // Reverse individual motors if needed so that
  // positive power makes all wheels spin in the correct direction
  mFL.set_reversed(REVERSED_FL);
  mFR.set_reversed(REVERSED_FR);
  mBL.set_reversed(REVERSED_BL);
  mBR.set_reversed(REVERSED_BR);


  if (IMU_PORT > 0) {
    imu.reset();   // Begin calibration (zeroing the gyro/accelerometer)
    // Wait up to ~2.5 seconds (250 × 10 ms) while the IMU calibrates
    // During this time, the IMU measures bias and stabilizes its sensors
    for (int t = 0; t < 250 && imu.is_calibrating(); ++t)
      pros::delay(10);  // Small delay to avoid blocking the CPU
  }
#endif
}

double heading_deg() {
#if IMU_PORT >= 0
  // Use the SAME API everywhere (get_heading is 0..360)
  return imu.get_heading();
#else
  return 0.0;
#endif
}

void zero_field_forward() {
#if IMU_PORT >= 0
  // Store the SAME quantity you’ll subtract later
  field_zero_raw = imu.get_heading();
#endif
}

// Normalize 4 wheel values to [-127..127]
static void normalize(double &fl, double &fr, double &bl, double &br) {
  const double maxmag = std::max({std::abs(fl), std::abs(fr), std::abs(bl), std::abs(br), 127.0});
  if (maxmag > 127.0) {
    const double k = 127.0 / maxmag;
    fl *= k; fr *= k; bl *= k; br *= k;
  }
}

void drive(int fwd, int str, int rot, bool field_centric) {
  fwd = deadband(fwd);
  str = deadband(str);
  rot = deadband(rot);

  double df = expo_scale(fwd);
  double ds = expo_scale(str);
  double dr = expo_scale(rot);

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

  // X-drive kinematics: +df=forward, +ds=right, +dr=CW
  double fl = df + ds + dr;
  double fr = df - ds - dr;
  double bl = df - ds + dr;
  double br = df + ds - dr;

  normalize(fl, fr, bl, br);

  mFL.move(static_cast<int>(fl));
  mFR.move(static_cast<int>(fr));
  mBL.move(static_cast<int>(bl));
  mBR.move(static_cast<int>(br));
}

// ---- Simple open-loop autonomous helpers ----
static void reset_positions() {
  #ifndef SIM
  mFL.tare_position(); mFR.tare_position();
  mBL.tare_position(); mBR.tare_position();
  #endif
}
static void move_all_relative(double fl, double fr, double bl, double br, int speed) {
  #ifndef SIM
  mFL.move_relative(fl, speed);
  mFR.move_relative(fr, speed);
  mBL.move_relative(bl, speed);
  mBR.move_relative(br, speed);
  #endif
}
static bool any_busy(double target_deg, double tol = 5.0) {
  #ifndef SIM
  const double T = std::max(0.0, std::abs(target_deg) - tol);
  return (std::abs(mFL.get_position()) < T) ||
         (std::abs(mFR.get_position()) < T) ||
         (std::abs(mBL.get_position()) < T) ||
         (std::abs(mBR.get_position()) < T);
  #else
  return false;
#endif
}

void drive_forward_deg(double wheel_deg, int speed) {
  reset_positions();
  move_all_relative(wheel_deg, wheel_deg, wheel_deg, wheel_deg, speed);
  #ifndef SIM
  pros::delay(10);
  while (any_busy(wheel_deg)) pros::delay(10);
  #endif
}
void strafe_right_deg(double wheel_deg, int speed) {
  reset_positions();
  move_all_relative(+wheel_deg, -wheel_deg, -wheel_deg, +wheel_deg, speed);
  #ifndef SIM
  pros::delay(10);
  while (any_busy(wheel_deg)) pros::delay(10);
  #endif
}
void turn_cw_deg(double wheel_deg, int speed) {
  reset_positions();
  move_all_relative(+wheel_deg, -wheel_deg, +wheel_deg, -wheel_deg, speed);
  #ifndef SIM
  pros::delay(10);
  while (any_busy(wheel_deg)) pros::delay(10);
  #endif
}

// ---------- LCD TELEMETRY ----------
#ifndef SIM
static pros::Task* telemetry_task = nullptr;
#endif

#ifndef SIM
static void telemetry_loop(void*) {
  pros::lcd::initialize(); // safe to call if already initialized
  while (true) {
    // Read commanded voltage (mV). Sign indicates direction.
    const double vFL = mFL.get_voltage();
    const double vFR = mFR.get_voltage();
    const double vBL = mBL.get_voltage();
    const double vBR = mBR.get_voltage();

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
    const double rFL = mFL.get_actual_velocity();
    const double rFR = mFR.get_actual_velocity();
    const double rBL = mBL.get_actual_velocity();
    const double rBR = mBR.get_actual_velocity();

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

void start_telemetry() {
  #ifndef SIM
  if (!telemetry_task) {
    telemetry_task = new pros::Task(telemetry_loop, nullptr, "xdrive-telemetry");
  }
  #endif
}

void stop_telemetry() {
  #ifndef SIM
  if (telemetry_task) {
    telemetry_task->remove();
    delete telemetry_task;
    telemetry_task = nullptr;
  }
  #endif
}

} // namespace xdrive
