#pragma once
#include "pros/apix.h"

namespace feeder {

  // All the patterns you described
  enum class Mode {
    Off,
    IntakeStore,
    ScoreLower,
    ScoreMiddle,
    ScoreUpper
  };

  void initialize();
  void set_mode(Mode m);
  void stop();   // convenience = set_mode(Off)

  // --- ADD: manual override for Feeder2 (used by auto-sort) ---
  void override_motor2(int pct);   // -127..127
  void clear_overrides();

} // namespace feeder
