#pragma once
#include "pros/apix.h"
#define XDRIVE_CORNER_MOTORS 2

// ====== CONFIGURE THESE ======
// ---- Drive Motor Ports ----
constexpr int PORT_FL1 = 3; // Motor 1
constexpr int PORT_FL2 = 4; // Motor 2
constexpr int PORT_FR1 = 5; // Motor 3
constexpr int PORT_FR2 = 6; // Motor 4
constexpr int PORT_BR1 = 7; // Motor 5
constexpr int PORT_BR2 = 8; // Motor 6
constexpr int PORT_BL1 = 9; // Motor 7
constexpr int PORT_BL2 = 10; // Motor 8

// ---- Drive Motor reversal ----
//Front left 1 & 2
constexpr bool REVERSED_FL1 = false;
constexpr bool REVERSED_FL2 = true;
//Front right 3 & 4
constexpr bool REVERSED_FR1 = true;
constexpr bool REVERSED_FR2 = false;
//Back right 5 & 6
constexpr bool REVERSED_BR1 = false;
constexpr bool REVERSED_BR2 = false;
//Back left 7 & 8
constexpr bool REVERSED_BL1 = true;
constexpr bool REVERSED_BL2 = true;

// ---- Mechanism Ports ----
constexpr int PORT_INTAKE  = 11;     // Intake motor port
constexpr int PORT_FEEDER_1_3  = 14; // Feeder motor 1 & 3 (tied together) front bottom and front middle
constexpr int PORT_FEEDER_2  = 19;   // Feeder motor back middle
constexpr int PORT_FEEDER_4  = 20;   // Feeder motor back top
constexpr int PORT_FEEDER_5  = 17;   // Feeder motor front top
constexpr int PORT_FEEDER_6  = 16;   // Feeder back bottom

// ---------- Mechanism options ----------
constexpr bool INTAKE_REVERSED = false;
// Feeder motor reversals
constexpr bool REVERSED_F_1_3 = true;
constexpr bool REVERSED_F_2 = true;
constexpr bool REVERSED_F_4 = true;
constexpr bool REVERSED_F_5 = true;
constexpr bool REVERSED_F_6 = false;

// ---- Sensors ----
constexpr int IMU_PORT = 13;
constexpr int PORT_OPTICAL = 12;     // VEX Optical Sensor (color sensor)
constexpr int TRACKING_WHEEL_PARALLEL_PORT = 15;
constexpr int TRACKING_WHEEL_PERPENDICULAR_PORT = 18;

// ---------- Color thresholds (tune on-field) ----------
enum class TeamColor { Red, Blue };
constexpr int PROX_THRESHOLD = 60;    // 0..255; higher = closer
constexpr double HUE_RED_MIN = 0.0,   HUE_RED_MAX = 25.0;
constexpr double HUE_BLUE_MIN = 190.0, HUE_BLUE_MAX = 250.0;

// ---------- Loader (Pneumatics) ----------
constexpr char LOADER_SOLENOID_PORT = 'A';
constexpr bool LOADER_SOLENOID_INVERTED = false; // Flip this if deploy/retract ends up backwards

// ---- Tracking wheel physical parameters ----
#define TRACKING_WHEEL_DIAMETER_IN 2.75     // inches
#define TRACKING_WHEEL_CIRCUMFERENCE_IN (TRACKING_WHEEL_DIAMETER_IN * M_PI)
#define TRACKING_TICKS_PER_REV 360.0        // if using shaft encoder
#define TRACKING_WHEEL_PARALLEL_OFFSET_IN  3.5  // distance from center to parallel wheel (forward +)
#define TRACKING_WHEEL_PERP_OFFSET_IN      4.25 // distance from center to perpendicular wheel (right +)

// ---- Gearing and encoder units ----
inline constexpr pros::motor_gearset_e_t DRIVE_GEARSET = pros::E_MOTOR_GEARSET_06;
inline constexpr pros::motor_encoder_units_e_t DRIVE_ENCODERS = pros::E_MOTOR_ENCODER_DEGREES;
/*
E_MOTOR_GEARSET_36  → Red cartridge (100 RPM, high torque)
E_MOTOR_GEARSET_18  → Green cartridge (200 RPM, standard)
E_MOTOR_GEARSET_06  → Blue cartridge (600 RPM, high speed)
---
E_MOTOR_ENCODER_DEGREES   → motor shaft position in degrees
E_MOTOR_ENCODER_ROTATIONS → position in full revolutions
E_MOTOR_ENCODER_COUNTS    → raw internal counts (ticks)
*/

// ---- Physical parameters (for distance-based moves) ----
#define WHEEL_DIAMETER_IN 2.75