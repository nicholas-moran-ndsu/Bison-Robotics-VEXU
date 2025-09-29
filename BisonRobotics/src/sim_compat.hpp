#pragma once

#ifdef SIM
  #include <cstdint>
  #include <cmath>

  // ---- Mock timing ----
  #include <chrono>
  #include <thread>
  inline uint32_t now_ms() {
    using namespace std::chrono;
    static auto t0 = steady_clock::now();
    return (uint32_t)duration_cast<milliseconds>(steady_clock::now() - t0).count();
  }
  inline void sleep_ms(uint32_t ms){ std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

  // ---- Motor mock (captures last .move() command in [-127..127]) ----
  struct MotorMock {
    int port; bool reversed=false;
    int last_cmd=0; // -127..127
    double sim_rpm=0.0;  // optional “measured” speed
    explicit MotorMock(int p, bool rev=false): port(p), reversed(rev) {}
    void set_gearing(int){};
    void set_encoder_units(int){};
    void set_reversed(bool r){ reversed=r; }
    void move(int v){ last_cmd = reversed ? -v : v; }
    void move_relative(double /*deg*/, int /*spd*/){ /*no-op*/ }
    void tare_position(){ }
    double get_position() const { return 0.0; }
    double get_voltage()  const { return last_cmd/127.0 * 12000.0; }
    double get_actual_velocity() const { return sim_rpm; }
  };

  struct ImuMock {
    double heading_deg = 0.0; // 0..360
    void reset(){ heading_deg = 0.0; }
    bool is_calibrating() const { return false; }
    double get_heading() const { return heading_deg; }
    double get_rotation() const { return heading_deg; }
  };

  inline double deg2rad(double d){ return d*M_PI/180.0; }

#else
  // ---- Real PROS adapters ----
  #include "api.h"
  inline uint32_t now_ms(){ return pros::millis(); }
  inline void sleep_ms(uint32_t ms){ pros::delay(ms); }
#endif