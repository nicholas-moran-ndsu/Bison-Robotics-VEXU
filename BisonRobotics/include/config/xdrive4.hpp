#pragma once
#define XDRIVE_CORNER_MOTORS 1

// ====== CONFIGURE THESE ======
// ---- Ports ----
#define PORT_FL1 2   //2
#define PORT_FR1 10  //10
#define PORT_BL1 11  //11
#define PORT_BR1 20  //20

// ---- Motor reversal ----
#define REVERSED_FL1 false
#define REVERSED_FR1 true
#define REVERSED_BL1 false
#define REVERSED_BR1 true

// ---- Sensors ----
#define IMU_PORT 6
#define TRACKING_WHEEL_PARALLEL_PORT 9
#define TRACKING_WHEEL_PERPENDICULAR_PORT 12

// ---- Tracking wheel physical parameters ----
#define TRACKING_WHEEL_DIAMETER_IN 2.75     // inches
#define TRACKING_WHEEL_CIRCUMFERENCE_IN (TRACKING_WHEEL_DIAMETER_IN * M_PI)
#define TRACKING_TICKS_PER_REV 360.0        // if using shaft encoder
#define TRACKING_WHEEL_PARALLEL_OFFSET_IN  5  // distance from center to parallel wheel (forward +)
#define TRACKING_WHEEL_PERP_OFFSET_IN      0 // distance from center to perpendicular wheel (right +)


// ---- Gearing and encoder units ----
#define GEARSET pros::E_MOTOR_GEARSET_18
#define ENCODERS pros::E_MOTOR_ENCODER_DEGREES
// (Other options: E_MOTOR_GEARSET_36, E_MOTOR_GEARSET_06)

// ---- Physical parameters (for distance-based moves) ----
#define WHEEL_DIAMETER_IN 4.0