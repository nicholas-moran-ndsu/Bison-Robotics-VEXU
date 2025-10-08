#pragma once
#include "pros/apix.h"

namespace feeder {
  void initialize();
  void set(int pct);          // -127..127
  void stop();
  void run_for_ms(int pct, uint32_t ms); // simple timed feed
}
