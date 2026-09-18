#pragma once
#include <string>

// @JSON_ENABLE
struct RegisterEvent {
  std::string deviceCode;
};

// @JSON_ENABLE
struct OldJoystickEvent {
  double right_x = 0;
  double right_y = 0;
  double left_x = 0;
  double left_y = 0;
};

// @JSON_ENABLE
struct SetSmootherEvent {
  bool enable = true;
};
