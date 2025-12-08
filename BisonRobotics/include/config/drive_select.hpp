#pragma once

// Pick one drive layout at compile-time
// You can set this here or pass -DXDRIVE4 / -DXDRIVE12 when building
#if !defined(XDRIVE4) && !defined(XDRIVE12)
  #define XDRIVE4 // default build
#endif

#if defined(XDRIVE4)
  #include "config/xdrive4.hpp"
#elif defined(XDRIVE12)
  #include "config/xdrive12.hpp"
#else
  #error "Define either XDRIVE4 or XDRIVE12 before including drive_select.hpp"
#endif
