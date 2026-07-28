// Requires libraries (Arduino Library Manager):
//   - "WebSockets" by Markus Sattler (Links2004/arduinoWebSockets)
//   - "ArduinoJson" by Benoit Blanchon
#include "WebSocketComm.h"
#include "Navigation.h"
#include "Mapping.h"
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

static WebSocketsClient ws;
static bool connected = false;
static unsigned long lastTelemetry = 0;
static unsigned long lastWifiAttempt = 0;

static void handleCommand(const char* payload, size_t len) {
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, payload, len);
  if (err) { DBG_PRINTLN("[WS] bad command JSON"); return; }

  const char* cmd = doc["command"] | "";
  DBG_PRINTF("[WS] command received: %s\n", cmd);

  if      (strcmp(cmd, "START") == 0)          Navigation::cmdStart();
  else if (strcmp(cmd, "STOP") == 0)           Navigation::cmdStop();
  else if (strcmp(cmd, "PAUSE") == 0)          Navigation::cmdPause();
  else if (strcmp(cmd, "RESUME") == 0)         Navigation::cmdResume();
  else if (strcmp(cmd, "EMERGENCY_STOP") == 0) Navigation::cmdEmergencyStop();
  else if (strcmp(cmd, "RESET_MAZE") == 0)     Navigation::cmdResetMaze();
  else if (strcmp(cmd, "CLEAR_MAP") == 0)      Navigation::cmdClearMap();
  else if (strcmp(cmd, "CALIBRATE") == 0)      Navigation::cmdCalibrate();
  else if (strcmp(cmd, "RETURN_HOME") == 0)    Navigation::cmdReturnHome();
  else if (strcmp(cmd, "SET_GOAL") == 0)       Navigation::cmdSetGoal(doc["x"] | DEFAULT_GOAL_X, doc["y"] | DEFAULT_GOAL_Y);

  // Acknowledge every command, per spec
  StaticJsonDocument<128> ack;
  ack["type"] = "ack";
  ack["command"] = cmd;
  ack["robotId"] = ROBOT_ID;
  String out;
  serializeJson(ack, out);
  ws.sendTXT(out);
}

static void onWsEvent(WStype_t type, uint8_t* payload, size_t len) {
  switch (type) {
    case WStype_CONNECTED:
      connected = true;
      DBG_PRINTLN("[WS] connected to backend bridge");
      {
        StaticJsonDocument<128> hello;
        hello["type"] = "hello";
        hello["robotId"] = ROBOT_ID;
        hello["token"] = ROBOT_AUTH_TOKEN;
        String out; serializeJson(hello, out);
        ws.sendTXT(out);
      }
      break;
    case WStype_DISCONNECTED:
      connected = false;
      DBG_PRINTLN("[WS] disconnected");
      break;
    case WStype_TEXT:
      handleCommand((const char*)payload, len);
      break;
    default:
      break;
  }
}

void WebSocketComm::init() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  DBG_PRINTLN("[WIFI] connecting...");
  ws.begin(WS_HOST, WS_PORT, WS_PATH);
  ws.onEvent(onWsEvent);
  ws.setReconnectInterval(WS_RECONNECT_INTERVAL_MS);
}

void WebSocketComm::loop() {
  if (WiFi.status() != WL_CONNECTED) {
    unsigned long now = millis();
    if (now - lastWifiAttempt > WIFI_RECONNECT_INTERVAL_MS) {
      lastWifiAttempt = now;
      WiFi.reconnect();
      DBG_PRINTLN("[WIFI] retrying connection...");
    }
    return;
  }
  ws.loop();

  unsigned long now = millis();
  if (now - lastTelemetry >= TELEMETRY_INTERVAL_MS) {
    lastTelemetry = now;
    sendTelemetry();
  }
}

bool WebSocketComm::isConnected() { return connected && WiFi.status() == WL_CONNECTED; }

void WebSocketComm::sendTelemetry() {
  if (!connected) return;
  const RobotStats &s = Navigation::getStats();

  StaticJsonDocument<512> doc;
  doc["type"] = "telemetry";
  doc["robotId"] = ROBOT_ID;
  doc["timestamp"] = (uint32_t)millis();
  doc["x"] = Navigation::getX();
  doc["y"] = Navigation::getY();
  char h[2] = { Navigation::getHeadingChar(), 0 };
  doc["heading"] = h;
  doc["state"] = Navigation::getStateName();
  doc["action"] = Navigation::getCurrentAction();
  doc["visitedCells"] = s.visitedCells;
  doc["deadEnds"] = s.deadEnds;
  doc["intersections"] = s.intersections;
  doc["turns"] = s.turns;
  doc["cellsTraveled"] = s.cellsTraveled;
  doc["totalDecisions"] = s.totalDecisions;
  doc["explorationPercent"] = Navigation::explorationPercent();
  doc["goalFound"] = s.goalFound;
  doc["missionComplete"] = s.missionComplete;
  doc["wifiRSSI"] = WiFi.RSSI();

  String out;
  serializeJson(doc, out);
  ws.sendTXT(out);
}

void WebSocketComm::sendEvent(const char* eventName) {
  StaticJsonDocument<128> doc;
  doc["type"] = "event";
  doc["robotId"] = ROBOT_ID;
  doc["event"] = eventName;
  String out; serializeJson(doc, out);
  ws.sendTXT(out);
  DBG_PRINTF("[WS] event sent: %s\n", eventName);
}

// Sends a single freshly-discovered cell (walls + flags) the moment the
// robot finishes sensing it, so the dashboard map fills in live during
// exploration instead of only appearing at the final snapshot.
void WebSocketComm::sendCellUpdate(int x, int y) {
  if (!connected) return;
  Cell &c = maze.at(x, y);
  StaticJsonDocument<256> doc;
  doc["type"] = "cell_update";
  doc["robotId"] = ROBOT_ID;
  doc["x"] = x; doc["y"] = y;
  doc["n"] = c.wallN; doc["s"] = c.wallS; doc["e"] = c.wallE; doc["w"] = c.wallW;
  doc["deadEnd"] = c.isDeadEnd;
  doc["intersection"] = c.isIntersection;
  doc["goal"] = c.isGoal;
  String out;
  serializeJson(doc, out);
  ws.sendTXT(out);
}

// Sends the full occupancy grid (walls only, for compact payload size) plus
// the computed shortest path, so the dashboard can render/freeze the final map.
void WebSocketComm::sendMapSnapshot() {
  DynamicJsonDocument doc(8192);
  doc["type"] = "map_snapshot";
  doc["robotId"] = ROBOT_ID;
  JsonArray cells = doc.createNestedArray("cells");
  for (int y = 0; y < maze.gridH(); y++) {
    for (int x = 0; x < maze.gridW(); x++) {
      Cell &c = maze.at(x, y);
      if (!c.visited) continue;
      JsonObject o = cells.createNestedObject();
      o["x"] = x; o["y"] = y;
      o["n"] = c.wallN; o["s"] = c.wallS; o["e"] = c.wallE; o["w"] = c.wallW;
      o["deadEnd"] = c.isDeadEnd;
      o["intersection"] = c.isIntersection;
      o["goal"] = c.isGoal;
    }
  }
  PathResult &p = Navigation::getShortestPath();
  JsonArray path = doc.createNestedArray("shortestPath");
  for (int i = 0; i < p.length; i++) {
    JsonObject o = path.createNestedObject();
    o["x"] = p.path[i].x; o["y"] = p.path[i].y;
  }
  doc["pathTurns"] = p.turns;
  doc["pathLength"] = p.length;

  String out;
  serializeJson(doc, out);
  ws.sendTXT(out);
}
