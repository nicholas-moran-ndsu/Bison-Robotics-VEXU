#pragma once
#define XDRIVE_CORNER_MOTORS 3

// ====== CONFIGURE THESE ======
// ---- Ports ----
#define PORT_FL1 4
#define PORT_FL2 5
#define PORT_FL3 6

#define PORT_FR1 1
#define PORT_FR2 2
#define PORT_FR3 3

#define PORT_BL1 7
#define PORT_BL2 8
#define PORT_BL3 9

#define PORT_BR1 10
#define PORT_BR2 11
#define PORT_BR3 12

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

// ---- Gearing and encoder units ----
#define GEARSET pros::E_MOTOR_GEARSET_06
#define ENCODERS pros::E_MOTOR_ENCODER_DEGREES
// (Other options: E_MOTOR_GEARSET_36, E_MOTOR_GEARSET_06)

// ---- Physical parameters (for distance-based moves) ----
#define WHEEL_DIAMETER_IN 2.75