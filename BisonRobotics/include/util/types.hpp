
#pragma once
#include <cstdint>

enum class DriveMode    : uint8_t { FieldCentric, RobotCentric };
enum class AutonRoutine : uint8_t { DoNothing, NearSide, FarSide, Skills };

//struct Pose { double x, y, theta; };   // used by odometry + autons