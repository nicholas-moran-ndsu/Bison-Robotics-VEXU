#pragma once
#define XDRIVE_CORNER_MOTORS 3

// ====== CONFIGURE THESE ======
// ---- Ports ----
#define PORT_FL1 4 // Motor 4
#define PORT_FL2 5 // Motor 5
#define PORT_FL3 13 // Motor 6

#define PORT_FR1 1 // Motor 1
#define PORT_FR2 2 // Motor 2
#define PORT_FR3 3 // Motor 3

#define PORT_BL1 7 // Motor 7
#define PORT_BL2 8 // Motor 8
#define PORT_BL3 9 // Motor 9

#define PORT_BR1 10 // Motor 10
#define PORT_BR2 11 // Motor 11
#define PORT_BR3 12 // Motor 12

// ---- Motor reversal ----
//Front left 4,5,6
#define REVERSED_FL1 false
#define REVERSED_FL2 true
#define REVERSED_FL3 false
//Front right 1,2,3
#define REVERSED_FR1 true
#define REVERSED_FR2 false
#define REVERSED_FR3 true
//Back left 7,8,9
#define REVERSED_BL1 false
#define REVERSED_BL2 true
#define REVERSED_BL3 false
//Back right 10,11,12
#define REVERSED_BR1 true
#define REVERSED_BR2 false
#define REVERSED_BR3 true

// ---- Sensors ----
#define IMU_PORT -1
#define TRACKING_WHEEL_PARALLEL_PORT -1
#define TRACKING_WHEEL_PERPENDICULAR_PORT -1

// ---- Tracking wheel physical parameters ----
#define TRACKING_WHEEL_DIAMETER_IN 2.75     // inches
#define TRACKING_WHEEL_CIRCUMFERENCE_IN (TRACKING_WHEEL_DIAMETER_IN * M_PI)
#define TRACKING_TICKS_PER_REV 360.0        // if using shaft encoder
#define TRACKING_WHEEL_PARALLEL_OFFSET_IN  3.5  // distance from center to parallel wheel (forward +)
#define TRACKING_WHEEL_PERP_OFFSET_IN      4.25 // distance from center to perpendicular wheel (right +)

// ---- Gearing and encoder units ----
#define GEARSET pros::E_MOTOR_GEARSET_06
#define ENCODERS pros::E_MOTOR_ENCODER_DEGREES
// (Other options: E_MOTOR_GEARSET_36, E_MOTOR_GEARSET_06)

// ---- Physical parameters (for distance-based moves) ----
#define WHEEL_DIAMETER_IN 2.75