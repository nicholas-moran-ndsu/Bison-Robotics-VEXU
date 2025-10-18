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
#define IMU_PORT 16

// ---- Gearing and encoder units ----
#define GEARSET pros::E_MOTOR_GEARSET_18
#define ENCODERS pros::E_MOTOR_ENCODER_DEGREES
// (Other options: E_MOTOR_GEARSET_36, E_MOTOR_GEARSET_06)

// ---- Physical parameters (for distance-based moves) ----
#define WHEEL_DIAMETER_IN 4.0