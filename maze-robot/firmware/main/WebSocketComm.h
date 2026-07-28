#pragma once
#include "config.h"

namespace WebSocketComm {
  void init();
  void loop();                       // call every main loop iteration
  void sendTelemetry();              // called on TELEMETRY_INTERVAL_MS cadence
  void sendEvent(const char* eventName);
  void sendCellUpdate(int x, int y);  // incremental: walls just discovered at (x,y)
  void sendMapSnapshot();            // full occupancy grid + shortest path
  bool isConnected();
}
