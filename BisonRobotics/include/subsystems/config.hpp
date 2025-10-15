#pragma once
#include "pros/apix.h"

// ---------- Ports ---------- =============== SET PORTS =================
constexpr int PORT_INTAKE  = 5;
constexpr int PORT_FEEDER  = 6;
constexpr int PORT_OPTICAL = 19;  // VEX Optical Sensor

// ---------- Motor options ----------
constexpr auto DRIVE_GEARSET = pros::E_MOTOR_GEARSET_18; // if needed
constexpr bool INTAKE_REVERSED = false;
constexpr bool FEEDER_REVERSED = false;

// ---------- Color thresholds (tune on-field) ----------
enum class TeamColor { Red, Blue };
constexpr int PROX_THRESHOLD = 60;    // 0..255; higher = closer
constexpr double HUE_RED_MIN = 0.0,   HUE_RED_MAX = 25.0;
constexpr double HUE_BLUE_MIN = 190.0, HUE_BLUE_MAX = 250.0;

// Controller mapping (change to taste)
constexpr auto BTN_INTAKE_IN   = pros::E_CONTROLLER_DIGITAL_R1;
constexpr auto BTN_INTAKE_OUT  = pros::E_CONTROLLER_DIGITAL_R2;
constexpr auto BTN_FEED_INDEX  = pros::E_CONTROLLER_DIGITAL_L1;
constexpr auto BTN_FEED_REVERSE= pros::E_CONTROLLER_DIGITAL_L2;
constexpr auto BTN_SORT_TOGGLE = pros::E_CONTROLLER_DIGITAL_A;
