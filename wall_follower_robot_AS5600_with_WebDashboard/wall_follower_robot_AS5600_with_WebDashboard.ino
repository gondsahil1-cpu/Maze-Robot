


/*
  ============================================================
  CENTERING WALL-FOLLOWER ROBOT — ESP32 + TB6612FNG
  ============================================================
  Behavior:
   - If a wall is detected on BOTH sides (corridor/maze), the
     robot compares left vs right distance and steers to stay
     centered between them.
   - If only the left wall is detected (open area on the right),
     it falls back to classic single-wall following at a fixed
     distance from the left wall.
   - A front sensor stops/turns the robot at dead ends.

  NEW IN THIS VERSION:
   - The ESP32 hosts its own WiFi network (SoftAP) and a small
     web dashboard showing live robot state, sensor distances,
     and wheel rotation (angle / revolutions / RPM per wheel).
     Connect to the AP below, then browse to the ESP32's IP.
   - Each AS5600 encoder's current position is captured at boot
     and treated as the 0° reference, so "angle" always restarts
     at 0 every time the robot is powered on.
  ============================================================
*/

// ---- TB6612FNG motor driver ----
#define PIN_STBY        13   // Standby (must be HIGH to drive motors)
#define PIN_AIN1        26   // Left motor  direction 1
#define PIN_AIN2        25   // Left motor  direction 2
#define PIN_PWMA        18   // Left motor  PWM
#define PIN_BIN1        27   // Right motor direction 1
#define PIN_BIN2        14   // Right motor direction 2
#define PIN_PWMB        19   // Right motor PWM

// ---- HC-SR04 ultrasonic sensors ----
#define PIN_TRIG_LEFT     4
#define PIN_ECHO_LEFT    22
#define PIN_TRIG_FRONT   5
#define PIN_ECHO_FRONT   21
#define PIN_TRIG_RIGHT   16
#define PIN_ECHO_RIGHT   17

// ---- AS5600 magnetic encoders (wheel odometry) ----
// Both AS5600 chips share the fixed I2C address 0x36, so each one
// gets its own hardware I2C bus (ESP32 has two: Wire and Wire1).
#include <Wire.h>
#define AS5600_ADDR       0x36
#define AS5600_REG_ANGLE  0x0E   // 12-bit filtered angle, 0-4095 counts/rev

#define PIN_ENC_L_SDA     32   // Left wheel encoder  (Wire  / I2C0)
#define PIN_ENC_L_SCL     33
#define PIN_ENC_R_SDA     15   // Right wheel encoder (Wire1 / I2C1)
#define PIN_ENC_R_SCL      0   // strapping pin — pull-up holds it HIGH, matches normal boot state

TwoWire I2C_ENC_R = TwoWire(1); // second hardware I2C peripheral

struct Encoder {
  TwoWire* bus;
  int32_t  lastRaw;     // last raw 0-4095 reading
  int64_t  totalCounts; // cumulative signed count, unwrapped across revolutions
  bool     ok;          // false if last read failed (magnet missing / wiring)
  uint16_t zeroRaw;     // raw angle captured at power-on — the "0 deg" reference
};

Encoder encL = { &Wire,      0, 0, false, 0 };
Encoder encR = { &I2C_ENC_R, 0, 0, false, 0 };

// ---- WiFi + web dashboard ----
#include <WiFi.h>
#include <WebServer.h>

const char* AP_SSID = "WallFollowerRobot";  // network the ESP32 creates
const char* AP_PASS = "robot1234";          // must be 8+ chars for WPA2

WebServer server(80);

// Snapshot of everything the dashboard shows, refreshed every loop() pass.
struct RobotStatus {
  String stateLabel  = "Starting...";
  String stateClass  = "stop";     // css class: center | left | turn | stop
  String mode        = "-";        // CENTERING | LEFT-WALL | OBSTACLE
  float  leftDist    = 0, rightDist = 0, frontDist = 0;
  bool   rightWallPresent = false;
  float  error       = 0;
  int    leftSpeed   = 0, rightSpeed = 0;
  bool   encLOk      = false, encROk = false;
  float  angleL      = 0, angleR = 0;   // degrees, relative to power-on position
  float  revL        = 0, revR   = 0;   // total revolutions since power-on
  float  rpmL        = 0, rpmR   = 0;
};
RobotStatus status;

// ---------------- Tunable parameters ----------------
const float MAX_DISTANCE_CM   = 30.0;  // treat anything beyond this as "no wall"
const float WALL_PRESENT_CM   = 10.0;  // right distance below this => "right wall exists" => centering mode
const float DESIRED_LEFT_CM   = 6.0;  // setpoint used ONLY in single-wall (left) mode
const float FRONT_STOP_CM     = 4.0;  // stop/turn if something this close in front

const int   BASE_SPEED  = 150;   // 0-255 PWM, forward cruising speed
const int   MAX_SPEED   = 220;
const int   MIN_SPEED   = 60;

// PID gains — tune these for your robot/sensors
float Kp = 8.0;
float Ki = 0.0;
float Kd = 1.0;

float integral = 0.0;
float lastError = 0.0;
unsigned long lastTime = 0;

// ---------------- Forward declarations ----------------
void handleRoot();
void handleData();
float encoderAngleDeg(const Encoder &enc);

// Reprints the dashboard address periodically so it doesn't get lost
// under the continuous sensor/encoder debug spam once the robot starts.
unsigned long lastDashboardReminder = 0;
void printDashboardBanner() {
  Serial.println();
  Serial.println("========================================");
  Serial.println("   WALL-FOLLOWER ROBOT — WEB DASHBOARD");
  Serial.println("========================================");
  Serial.print  ("  WiFi network : "); Serial.println(AP_SSID);
  Serial.print  ("  WiFi password: "); Serial.println(AP_PASS);
  Serial.print  ("  Dashboard URL: http://"); Serial.println(WiFi.softAPIP());
  Serial.println("========================================");
  Serial.println();
}

// ---------------- Setup ----------------
void setup() {
  // Bring Serial up first and give the monitor a moment to attach so
  // the WiFi banner below isn't missed.
  Serial.begin(115200);
  delay(1500);

  // ---- STEP 1: connect WiFi FIRST, before anything else starts ----
  Serial.println("Starting WiFi access point...");
  WiFi.mode(WIFI_AP);
  bool apOk = WiFi.softAP(AP_SSID, AP_PASS);

  // softAP() is normally instant, but wait/retry briefly just in case
  // so we never fall through to "robot start" without a valid IP.
  unsigned long apWaitStart = millis();
  while ((!apOk || WiFi.softAPIP() == IPAddress(0, 0, 0, 0)) && millis() - apWaitStart < 5000) {
    delay(200);
    apOk = WiFi.softAP(AP_SSID, AP_PASS);
  }

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();

  printDashboardBanner();
  lastDashboardReminder = millis();

  // ---- STEP 2: now that WiFi + the dashboard are up, start the robot ----
  Serial.println("WiFi ready — starting robot hardware...");

  pinMode(PIN_TRIG_LEFT, OUTPUT);  pinMode(PIN_ECHO_LEFT, INPUT);
  pinMode(PIN_TRIG_RIGHT, OUTPUT); pinMode(PIN_ECHO_RIGHT, INPUT);
  pinMode(PIN_TRIG_FRONT, OUTPUT); pinMode(PIN_ECHO_FRONT, INPUT);

  pinMode(PIN_STBY, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT); pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT); pinMode(PIN_PWMB, OUTPUT);

  digitalWrite(PIN_STBY, HIGH); // enable the driver (LOW = low-power standby, motors off)

  Wire.begin(PIN_ENC_L_SDA, PIN_ENC_L_SCL, 400000);
  I2C_ENC_R.begin(PIN_ENC_R_SDA, PIN_ENC_R_SCL, 400000);

  // Prime lastRaw so the first update() doesn't see a huge fake jump,
  // AND capture that same reading as zeroRaw so the wheel's current
  // physical position becomes the 0° reference for THIS power cycle.
  uint16_t r0 = readAS5600Raw(*encL.bus);
  if (r0 != 0xFFFF) { encL.lastRaw = r0; encL.zeroRaw = r0; encL.ok = true; }
  uint16_t r1 = readAS5600Raw(*encR.bus);
  if (r1 != 0xFFFF) { encR.lastRaw = r1; encR.zeroRaw = r1; encR.ok = true; }
  Serial.println("AS5600: wheel angles zeroed at current position (boot reference = 0 deg).");

  lastTime = millis();
}

// ---------------- AS5600 encoder reading ----------------
// Returns raw angle 0-4095, or 0xFFFF on I2C error (no ACK, magnet missing, etc.)
uint16_t readAS5600Raw(TwoWire &bus) {
  bus.beginTransmission(AS5600_ADDR);
  bus.write(AS5600_REG_ANGLE);
  if (bus.endTransmission(false) != 0) return 0xFFFF;
  if (bus.requestFrom((int)AS5600_ADDR, 2) != 2) return 0xFFFF;
  uint16_t hi = bus.read();
  uint16_t lo = bus.read();
  return ((hi << 8) | lo) & 0x0FFF;
}

// Updates cumulative count for one encoder, handling 0/4095 wraparound
void updateEncoder(Encoder &enc) {
  uint16_t raw = readAS5600Raw(*enc.bus);
  if (raw == 0xFFFF) { enc.ok = false; return; }
  enc.ok = true;

  int32_t diff = (int32_t)raw - enc.lastRaw;
  if (diff > 2048)  diff -= 4096;   // wrapped forward past 4095->0
  if (diff < -2048) diff += 4096;   // wrapped backward past 0->4095

  enc.totalCounts += diff;
  enc.lastRaw = raw;
}

// Wheel angle in degrees, relative to the position captured at power-on (0-360).
float encoderAngleDeg(const Encoder &enc) {
  int32_t rel = (int32_t)enc.lastRaw - (int32_t)enc.zeroRaw;
  rel %= 4096;
  if (rel < 0) rel += 4096;
  return rel * (360.0 / 4096.0);
}

// ---------------- Ultrasonic read (cm), with timeout ----------------
float readDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // 25000us timeout ~ 4m range; returns 0 if no echo (out of range)
  long duration = pulseIn(echoPin, HIGH, 25000UL);
  if (duration == 0) return MAX_DISTANCE_CM; // no echo -> treat as far / no wall

  float distance = (duration * 0.0343) / 2.0; // speed of sound = 343 m/s
  if (distance > MAX_DISTANCE_CM) distance = MAX_DISTANCE_CM;
  return distance;
}

// ---------------- Motor control (TB6612FNG) ----------------
// speed: -255..255 (negative = reverse, 0 = coast/stop)
void setMotor(int in1, int in2, int pwmPin, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW); // coast
  }

  analogWrite(pwmPin, abs(speed)); // 0-255 duty cycle
}

void setMotors(int leftSpeed, int rightSpeed) {
  setMotor(PIN_AIN1, PIN_AIN2, PIN_PWMA, leftSpeed);
  setMotor(PIN_BIN1, PIN_BIN2, PIN_PWMB, rightSpeed);
}

void stopMotors() {
  setMotors(0, 0);
}

// ---------------- Web dashboard ----------------
const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Wall Follower Robot</title>
<style>
  body { font-family: -apple-system, Arial, sans-serif; background:#111; color:#eee; margin:0; padding:16px; }
  h1 { font-size:1.25em; text-align:center; color:#4fd1c5; margin-bottom:16px; }
  .grid { display:grid; grid-template-columns: repeat(auto-fit, minmax(220px,1fr)); gap:12px; max-width:900px; margin:0 auto; }
  .card { background:#1c1c1c; border-radius:10px; padding:14px 16px; box-shadow:0 2px 6px rgba(0,0,0,.4); }
  .card h2 { font-size:0.9em; margin:0 0 8px; color:#4fd1c5; text-transform:uppercase; letter-spacing:.5px; }
  .row { display:flex; justify-content:space-between; padding:3px 0; font-size:0.95em; border-bottom:1px solid #2a2a2a; }
  .row:last-child { border-bottom:none; }
  .val { font-weight:600; color:#fff; }
  .state { text-align:center; font-size:1.3em; font-weight:700; padding:10px; border-radius:8px; margin:0 auto 16px; max-width:900px; }
  .state.center { background:#22543d; color:#9ae6b4; }
  .state.left   { background:#2c5282; color:#bee3f8; }
  .state.turn   { background:#7b341e; color:#fbd38d; }
  .state.stop   { background:#742a2a; color:#feb2b2; }
  .ok  { color:#68d391; }
  .err { color:#fc8181; }
  footer { text-align:center; margin-top:16px; font-size:0.75em; color:#666; }
</style>
</head>
<body>
<h1>Wall Follower Robot &mdash; Live Status</h1>
<div id="state" class="state">Loading...</div>
<div class="grid">
  <div class="card">
    <h2>Distance Sensors</h2>
    <div class="row"><span>Left</span><span class="val" id="dl">--</span></div>
    <div class="row"><span>Front</span><span class="val" id="df">--</span></div>
    <div class="row"><span>Right</span><span class="val" id="dr">--</span></div>
    <div class="row"><span>Right wall detected</span><span class="val" id="rw">--</span></div>
  </div>
  <div class="card">
    <h2>Steering / PID</h2>
    <div class="row"><span>Mode</span><span class="val" id="mode">--</span></div>
    <div class="row"><span>Error</span><span class="val" id="err">--</span></div>
    <div class="row"><span>Left motor speed</span><span class="val" id="ls">--</span></div>
    <div class="row"><span>Right motor speed</span><span class="val" id="rs">--</span></div>
  </div>
  <div class="card">
    <h2>Left Wheel (AS5600)</h2>
    <div class="row"><span>Status</span><span class="val" id="lok">--</span></div>
    <div class="row"><span>Angle (since power-on)</span><span class="val" id="langle">--</span></div>
    <div class="row"><span>Revolutions</span><span class="val" id="lrev">--</span></div>
    <div class="row"><span>RPM</span><span class="val" id="lrpm">--</span></div>
  </div>
  <div class="card">
    <h2>Right Wheel (AS5600)</h2>
    <div class="row"><span>Status</span><span class="val" id="rok">--</span></div>
    <div class="row"><span>Angle (since power-on)</span><span class="val" id="rangle">--</span></div>
    <div class="row"><span>Revolutions</span><span class="val" id="rrev">--</span></div>
    <div class="row"><span>RPM</span><span class="val" id="rrpm">--</span></div>
  </div>
</div>
<footer>Auto-refreshing every 400&nbsp;ms &middot; served from the ESP32's own WiFi access point</footer>
<script>
async function poll(){
  try{
    const r = await fetch('/data');
    const d = await r.json();
    const st = document.getElementById('state');
    st.textContent = d.state;
    st.className = 'state ' + d.stateClass;
    document.getElementById('dl').textContent = d.leftDist.toFixed(1) + ' cm';
    document.getElementById('df').textContent = d.frontDist.toFixed(1) + ' cm';
    document.getElementById('dr').textContent = d.rightDist.toFixed(1) + ' cm';
    document.getElementById('rw').textContent = d.rightWallPresent ? 'Yes' : 'No';
    document.getElementById('mode').textContent = d.mode;
    document.getElementById('err').textContent = d.error.toFixed(2);
    document.getElementById('ls').textContent = d.leftSpeed;
    document.getElementById('rs').textContent = d.rightSpeed;
    document.getElementById('lok').innerHTML = d.encLOk ? '<span class="ok">OK</span>' : '<span class="err">ERROR</span>';
    document.getElementById('langle').textContent = d.angleL.toFixed(1) + ' deg';
    document.getElementById('lrev').textContent = d.revL.toFixed(2);
    document.getElementById('lrpm').textContent = d.rpmL.toFixed(1);
    document.getElementById('rok').innerHTML = d.encROk ? '<span class="ok">OK</span>' : '<span class="err">ERROR</span>';
    document.getElementById('rangle').textContent = d.angleR.toFixed(1) + ' deg';
    document.getElementById('rrev').textContent = d.revR.toFixed(2);
    document.getElementById('rrpm').textContent = d.rpmR.toFixed(1);
  }catch(e){ /* ignore a dropped poll, try again */ }
  setTimeout(poll, 400);
}
poll();
</script>
</body>
</html>
)HTML";

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleData() {
  String json = "{";
  json += "\"state\":\""      + status.stateLabel + "\",";
  json += "\"stateClass\":\"" + status.stateClass + "\",";
  json += "\"leftDist\":"     + String(status.leftDist, 2) + ",";
  json += "\"rightDist\":"    + String(status.rightDist, 2) + ",";
  json += "\"frontDist\":"    + String(status.frontDist, 2) + ",";
  json += "\"rightWallPresent\":" + String(status.rightWallPresent ? "true" : "false") + ",";
  json += "\"mode\":\""       + status.mode + "\",";
  json += "\"error\":"        + String(status.error, 2) + ",";
  json += "\"leftSpeed\":"    + String(status.leftSpeed) + ",";
  json += "\"rightSpeed\":"   + String(status.rightSpeed) + ",";
  json += "\"encLOk\":"       + String(status.encLOk ? "true" : "false") + ",";
  json += "\"encROk\":"       + String(status.encROk ? "true" : "false") + ",";
  json += "\"angleL\":"       + String(status.angleL, 2) + ",";
  json += "\"angleR\":"       + String(status.angleR, 2) + ",";
  json += "\"revL\":"         + String(status.revL, 3) + ",";
  json += "\"revR\":"         + String(status.revR, 3) + ",";
  json += "\"rpmL\":"         + String(status.rpmL, 2) + ",";
  json += "\"rpmR\":"         + String(status.rpmR, 2);
  json += "}";
  server.send(200, "application/json", json);
}

// ---------------- Main loop ----------------
void loop() {
  server.handleClient(); // service the dashboard — cheap no-op if nobody's connected

  // Reprint the dashboard address every 10s so it's easy to spot even
  // if the Serial Monitor was opened after boot and missed the banner.
  if (millis() - lastDashboardReminder > 10000) {
    printDashboardBanner();
    lastDashboardReminder = millis();
  }

  float leftDist  = readDistanceCM(PIN_TRIG_LEFT,  PIN_ECHO_LEFT);
  float rightDist = readDistanceCM(PIN_TRIG_RIGHT, PIN_ECHO_RIGHT);
  float frontDist = readDistanceCM(PIN_TRIG_FRONT, PIN_ECHO_FRONT);

  // --- Dead-end / obstacle handling ---
  if (frontDist < FRONT_STOP_CM) {
    status.leftDist  = leftDist;
    status.rightDist = rightDist;
    status.frontDist = frontDist;
    status.mode       = "OBSTACLE";
    status.stateLabel = "Turning at obstacle";
    status.stateClass = "turn";

    stopMotors();
    delay(150);
    // Turn toward the side with more room
    if (rightDist > leftDist) {
      setMotors(150, -150);  // pivot right
      status.leftSpeed = 150; status.rightSpeed = -150;
    } else {
      setMotors(-150, 150);  // pivot left
      status.leftSpeed = -150; status.rightSpeed = 150;
    }
    delay(300);
    integral = 0; lastError = 0; // reset PID after a hard maneuver
    return;
  }

  // --- Choose mode and compute error ---
  bool rightWallPresent = (rightDist < WALL_PRESENT_CM);
  float error;

  if (rightWallPresent) {
    // CENTERING MODE: balance distance to both walls.
    // error > 0  => closer to RIGHT wall => steer left
    // error < 0  => closer to LEFT wall  => steer right
    error = leftDist - rightDist;
  } else {
    // SINGLE-WALL (LEFT) FOLLOWING MODE:
    // too close to left wall => steer right (away from it)
    // too far from left wall => steer left (toward it)
    error = -(DESIRED_LEFT_CM - leftDist); // sign convention matched to centering mode
  }

  // --- PID ---
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt <= 0) dt = 0.001;

  // --- Encoder update (wheel odometry) ---
  int64_t prevCountsL = encL.totalCounts;
  int64_t prevCountsR = encR.totalCounts;
  updateEncoder(encL);
  updateEncoder(encR);
  const float COUNTS_PER_REV = 4096.0;
  float rpmL = ((encL.totalCounts - prevCountsL) / COUNTS_PER_REV) * (60.0 / dt);
  float rpmR = ((encR.totalCounts - prevCountsR) / COUNTS_PER_REV) * (60.0 / dt);

  integral += error * dt;
  integral = constrain(integral, -50, 50); // anti-windup clamp
  float derivative = (error - lastError) / dt;

  float correction = Kp * error + Ki * integral + Kd * derivative;

  lastError = error;
  lastTime = now;

  // correction > 0 => steer left (slow left wheel, speed up right wheel)
  // correction < 0 => steer right
  int leftSpeed  = BASE_SPEED - correction;
  int rightSpeed = BASE_SPEED + correction;

  leftSpeed  = constrain(leftSpeed,  MIN_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

  setMotors(leftSpeed, rightSpeed);

  // --- Update dashboard snapshot ---
  status.leftDist        = leftDist;
  status.rightDist       = rightDist;
  status.frontDist       = frontDist;
  status.rightWallPresent = rightWallPresent;
  status.mode            = rightWallPresent ? "CENTERING" : "LEFT-WALL";
  status.stateLabel       = rightWallPresent ? "Following: centering" : "Following: left wall";
  status.stateClass       = rightWallPresent ? "center" : "left";
  status.error            = error;
  status.leftSpeed        = leftSpeed;
  status.rightSpeed       = rightSpeed;
  status.encLOk           = encL.ok;
  status.encROk           = encR.ok;
  status.angleL           = encoderAngleDeg(encL);
  status.angleR           = encoderAngleDeg(encR);
  status.revL             = encL.totalCounts / COUNTS_PER_REV;
  status.revR             = encR.totalCounts / COUNTS_PER_REV;
  status.rpmL             = rpmL;
  status.rpmR             = rpmR;

  // --- Debug ---
  Serial.print("L:"); Serial.print(leftDist);
  Serial.print(" R:"); Serial.print(rightDist);
  Serial.print(" F:"); Serial.print(frontDist);
  Serial.print(" Mode:"); Serial.print(rightWallPresent ? "CENTER" : "LEFT-ONLY");
  Serial.print(" err:"); Serial.print(error);
  Serial.print(" Lspd:"); Serial.print(leftSpeed);
  Serial.print(" Rspd:"); Serial.print(rightSpeed);
  Serial.print(" | EncL:"); Serial.print(encL.ok ? String(encL.totalCounts) : "ERR");
  Serial.print(" AngL:"); Serial.print(status.angleL, 1);
  Serial.print(" RPM:"); Serial.print(rpmL, 1);
  Serial.print(" EncR:"); Serial.print(encR.ok ? String(encR.totalCounts) : "ERR");
  Serial.print(" AngR:"); Serial.print(status.angleR, 1);
  Serial.print(" RPM:"); Serial.println(rpmR, 1);

  delay(20); // small loop delay for sensor settling
}
