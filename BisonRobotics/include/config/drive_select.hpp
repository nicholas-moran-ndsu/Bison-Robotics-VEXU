#pragma once

// Pick one drive layout at compile-time
// You can set this here or pass -DXDRIVE4 / -DXDRIVE12 when building
#if !defined(XDRIVE4) && !defined(XDRIVE12) && !defined(XDRIVE8)
  #define XDRIVE8 // default build
#endif

#if defined(XDRIVE8)
  #include "config/xdrive8.hpp"
#elif defined(XDRIVE12)
  #include "config/xdrive12.hpp"
#else
  #error "Define XDRIVE8, XDRIVE12, or XDRIVE4 before including drive_select.hpp"
#endif
