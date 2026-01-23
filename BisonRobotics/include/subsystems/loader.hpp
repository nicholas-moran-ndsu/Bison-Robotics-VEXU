#pragma once
#include "pros/adi.hpp"

namespace loader {
  void initialize();

  // Explicit control
  void deploy();
  void retract();
  void set_deployed(bool deployed);

  // State
  bool is_deployed();
}
