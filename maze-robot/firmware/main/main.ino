// =============================================================================
//  Autonomous Maze-Solving Robot — Main Firmware
//  ESP32 + TB6612FNG + 2x N20 + 3x HC-SR04 + Buzzer + WiFi
//
//  Architecture (see docs/ARCHITECTURE.md):
//    Sensors        -> raw ultrasonic distance reads (median-filtered)
//    Mapping        -> occupancy grid (walls, visited, dead-ends, flood fill)
//    PathPlanner    -> A* shortest-path solver over the known grid
//    Navigation     -> finite-state machine: EXPLORING -> MAZE_COMPLETE ->
//                      PLANNING -> RUNNING_PATH -> CELEBRATING -> IDLE
//                      Uses non-blocking millis()-timed motion primitives.
//    WebSocketComm  -> WiFi + WebSocket telemetry (100ms) & command intake
//    Motors         -> TB6612FNG low-level drive
//
//  This sketch itself only wires the modules together and must stay free of
//  any blocking delay() in loop() — all timing lives inside Navigation's
//  motion-primitive queue and WebSocketComm's internal cadence checks.
// =============================================================================
#include "config.h"
#include "Motors.h"
#include "Sensors.h"
#include "Mapping.h"
#include "PathPlanner.h"
#include "Navigation.h"
#include "WebSocketComm.h"

void setup() {
  Serial.begin(115200);
  delay(200);
  DBG_PRINTLN("=== Autonomous Maze Robot booting ===");

  pinMode(BUZZER_PIN, OUTPUT);
#ifdef STATUS_LED_PIN
  pinMode(STATUS_LED_PIN, OUTPUT);
#endif

  Motors::init();
  Motors::stop();
  Sensors::init();
  Navigation::init();
  WebSocketComm::init();

  DBG_PRINTLN("[SETUP] complete, entering main loop");
}

void loop() {
  unsigned long now = millis();

  WebSocketComm::loop();     // WiFi/WS reconnection, telemetry cadence, command intake
  Navigation::tick(now);     // advance the robot's FSM + motion queue (non-blocking)

#ifdef STATUS_LED_PIN
  // Simple heartbeat / status LED (future-ready for richer patterns)
  digitalWrite(STATUS_LED_PIN, WebSocketComm::isConnected() ? HIGH : (now / 250) % 2);
#endif
}
