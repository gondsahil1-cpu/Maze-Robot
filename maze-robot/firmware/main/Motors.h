#pragma once
#include "config.h"

namespace Motors {
  void init();
  void stop();
  void forward(int speed);
  void reverse(int speed);
  void differential(int left, int right);
  void pivotLeft(int speed);
  void pivotRight(int speed);
}
