// include/util/types.hpp
#pragma once
#include <cstdint>

enum class DriveMode : uint8_t { FieldCentric, RobotCentric };
enum class AutonRoutine : uint8_t { DoNothing, NearSide, FarSide, Skills };
