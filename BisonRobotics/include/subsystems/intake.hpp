#pragma once
#include "pros/apix.h"

namespace intake {
  void initialize();
  void set(int pct);      // -127..127
  void stop();
}
