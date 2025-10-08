#pragma once
#include <cstdint>

enum class DriveMode : uint8_t { FieldCentric, RobotCentric };
enum class AutonRoutine : uint8_t { DoNothing, NearSide, FarSide, Skills };

namespace selector {
  void init();

  void ui_loop_once();

  // Current mode
  DriveMode drive_mode();
  AutonRoutine auton();

  // Pretty names for screens/logs
  const char* drive_mode_name(DriveMode m);
  const char* auton_name(AutonRoutine a);
}
