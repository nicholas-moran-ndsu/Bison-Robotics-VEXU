#pragma once
#include <cstdint>

// ===================== Drive =====================
enum class DriveMode : uint8_t {
  FieldCentric,
  RobotCentric
};

// ===================== Autonomous =====================
enum class AutonRoutine : uint8_t {
  Red_Left,
  Red_Right,
  Blue_Left,
  Blue_Right,
  Skills
};

