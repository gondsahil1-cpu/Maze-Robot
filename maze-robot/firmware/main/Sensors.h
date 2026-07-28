#pragma once
#include "config.h"

struct SensorReading {
  float front, left, right;
  bool wallFront, wallLeft, wallRight;
};

namespace Sensors {
  void init();
  // Blocking single-ping read (median-of-3), used only at decision points.
  SensorReading readAll();
  // Fast single front ping, used as a safety check during forward travel.
  float readFrontFast();
}
