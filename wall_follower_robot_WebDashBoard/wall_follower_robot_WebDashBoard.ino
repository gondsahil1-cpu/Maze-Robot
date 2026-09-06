


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
  float  distanceTraveledM = 0;         // estimated from wheel odometry
  bool   isMoving    = false;
};
RobotStatus status;

// ---------------- Tunable parameters ----------------
const float MAX_DISTANCE_CM   = 30.0;  // treat anything beyond this as "no wall"
const float WALL_PRESENT_CM   = 10.0;  // right distance below this => "right wall exists" => centering mode
const float DESIRED_LEFT_CM   = 6.0;  // setpoint used ONLY in single-wall (left) mode
const float FRONT_STOP_CM     = 4.0;  // stop/turn if something this close in front

const int   BASE_SPEED  = 200;   // 0-255 PWM, forward cruising speed
const int   MAX_SPEED   = 220;
const int   MIN_SPEED   = 60;

const float WHEEL_DIAMETER_CM = 6.5; // measure your actual wheel and set this
                                      // for an accurate "distance traveled" reading

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
  WiFi.setSleep(false);  // disable WiFi power-save — sleep mode is a common
                         // cause of a SoftAP going unresponsive/timing out
                         // after it's been idle for a bit
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

// Like delay(), but keeps servicing the web dashboard in small slices
// instead of blocking it for the whole duration.
void serverDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    server.handleClient();
    delay(5);
  }
}

// ---------------- Web dashboard ----------------
const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>WallBot Dashboard</title>
<style>
  :root{
    --teal:#4fd8c4; --yellow:#f2c94c; --blue:#5b9cf6; --green:#34d399; --red:#f2637a;
    --glass:rgba(255,255,255,.06); --glass-strong:rgba(255,255,255,.09);
    --border:rgba(255,255,255,.12); --text:#f5f7fa; --dim:rgba(245,247,250,.55);
  }
  *{box-sizing:border-box;}
  body{
    margin:0; padding:24px; min-height:100vh;
    font-family:-apple-system,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;
    color:var(--text);
    background:
      radial-gradient(circle at 15% -10%, #3d3d42 0%, transparent 55%),
      radial-gradient(circle at 100% 10%, #2c2c30 0%, transparent 45%),
      linear-gradient(180deg,#1b1b1e 0%,#0d0d0f 100%);
    background-attachment:fixed;
  }
  .wrap{max-width:1080px;margin:0 auto;}
  .header{display:flex;justify-content:space-between;align-items:center;margin-bottom:22px;flex-wrap:wrap;gap:14px;}
  .brand h1{font-size:1.7em;font-weight:800;letter-spacing:-.02em;color:var(--teal);margin:0;}
  .brand p{margin:2px 0 0;font-size:.82em;color:var(--dim);}
  .header-right{display:flex;align-items:center;gap:12px;}
  .status-pill{display:flex;align-items:center;gap:8px;background:var(--glass);border:1px solid var(--border);
    padding:9px 16px;border-radius:999px;font-size:.8em;font-weight:600;}
  .status-dot{width:8px;height:8px;border-radius:50%;background:var(--green);box-shadow:0 0 8px var(--green);animation:pulse 1.6s infinite;}
  .status-dot.lost{background:var(--red);box-shadow:0 0 8px var(--red);animation:none;}
  @keyframes pulse{0%,100%{opacity:1;}50%{opacity:.35;}}
  .avatar{width:42px;height:42px;border-radius:50%;background:linear-gradient(135deg,#2c5282,#16283f);
    display:flex;align-items:center;justify-content:center;font-size:1.25em;border:1px solid var(--border);}

  .state-banner{text-align:center;font-size:1.1em;font-weight:700;padding:15px;border-radius:18px;
    margin:0 0 20px;border:1px solid transparent;}
  .state-banner.center{background:rgba(79,216,196,.14);color:#8ff2de;border-color:rgba(79,216,196,.35);}
  .state-banner.left  {background:rgba(91,156,246,.14);color:#aecdfb;border-color:rgba(91,156,246,.35);}
  .state-banner.turn  {background:rgba(242,201,76,.14);color:#f7e0a0;border-color:rgba(242,201,76,.35);}
  .state-banner.stop  {background:rgba(242,99,122,.14);color:#f9adb9;border-color:rgba(242,99,122,.35);}

  .top-grid{display:grid;grid-template-columns:1.3fr 1fr 1.5fr;gap:16px;margin-bottom:16px;}
  .bottom-grid{display:grid;grid-template-columns:1.6fr 1fr;gap:16px;}
  @media (max-width:860px){ .top-grid,.bottom-grid{grid-template-columns:1fr;} }

  .card{background:var(--glass);border:1px solid var(--border);border-radius:22px;padding:20px;
    backdrop-filter:blur(18px); -webkit-backdrop-filter:blur(18px);}
  .card-head{display:flex;justify-content:space-between;align-items:baseline;margin-bottom:16px;}
  .card-head h2{font-size:.95em;font-weight:700;color:var(--teal);margin:0;}
  .card-head .hint{font-size:.72em;color:var(--dim);}

  /* distance bar chart */
  .bars{display:flex;justify-content:space-around;align-items:flex-end;height:170px;}
  .bar-col{display:flex;flex-direction:column;align-items:center;justify-content:flex-end;height:100%;width:64px;}
  .bar-val{font-size:.8em;font-weight:700;margin-bottom:8px;}
  .bar{width:32px;border-radius:16px;min-height:8px;transition:height .3s ease;
    background:linear-gradient(180deg,rgba(255,255,255,.4),rgba(255,255,255,.14));}
  .bar.alert{background:linear-gradient(180deg,#ffb3bd,var(--red));}
  .bar-label{margin-top:10px;font-size:.75em;color:var(--dim);}

  /* stat stack */
  .stat-stack{display:flex;flex-direction:column;gap:14px;height:100%;justify-content:center;}
  .stat-card{display:flex;align-items:center;gap:14px;background:var(--glass-strong);
    border:1px solid var(--border);border-radius:16px;padding:14px 16px;}
  .stat-icon{width:40px;height:40px;border-radius:50%;background:rgba(255,255,255,.08);
    display:flex;align-items:center;justify-content:center;flex-shrink:0;}
  .stat-icon svg{width:19px;height:19px;stroke:var(--teal);fill:none;stroke-width:2;stroke-linecap:round;}
  .stat-value{font-size:1.1em;font-weight:800;line-height:1.1;}
  .stat-label{font-size:.72em;color:var(--dim);margin-top:2px;}

  /* overview donut */
  .donut-wrap{position:relative;width:148px;height:148px;margin:6px auto 20px;}
  .donut-wrap svg{width:100%;height:100%;}
  .donut-track{fill:none;stroke:rgba(255,255,255,.08);stroke-width:14;}
  .donut-seg{fill:none;stroke-width:14;stroke-linecap:round;transition:stroke-dasharray .3s ease;}
  .donut-center{position:absolute;inset:0;display:flex;flex-direction:column;align-items:center;justify-content:center;}
  .donut-pct{font-size:1.6em;font-weight:800;}
  .donut-sub{font-size:.7em;color:var(--dim);margin-top:2px;text-align:center;padding:0 10px;}
  .overview-list .o-row{display:flex;align-items:center;gap:10px;padding:9px 0;border-top:1px solid rgba(255,255,255,.07);}
  .overview-list .o-row:first-child{border-top:none;}
  .o-dot{width:9px;height:9px;border-radius:50%;flex-shrink:0;}
  .o-body{flex:1;min-width:0;}
  .o-label{font-size:.78em;color:var(--dim);}
  .o-value{font-size:.95em;font-weight:700;}
  .o-sub{font-size:.72em;margin-top:1px;}

  /* wheel health list */
  .health-row{display:flex;align-items:center;gap:14px;padding:13px 0;border-top:1px solid rgba(255,255,255,.07);}
  .health-row:first-child{border-top:none;}
  .health-ring{width:36px;height:36px;border-radius:50%;border:2px solid var(--teal);flex-shrink:0;
    display:flex;align-items:center;justify-content:center;font-weight:700;color:var(--teal);}
  .health-ring.err{border-color:var(--red);color:var(--red);}
  .health-info{flex:1;min-width:0;}
  .health-title{font-weight:700;font-size:.95em;}
  .health-sub{font-size:.75em;color:var(--dim);margin-top:2px;}
  .badge{padding:7px 15px;border-radius:999px;font-size:.75em;font-weight:700;white-space:nowrap;flex-shrink:0;}
  .badge.ok{background:rgba(52,211,153,.16);color:var(--green);}
  .badge.err{background:rgba(242,99,122,.16);color:var(--red);}
  .badge.warn{background:rgba(242,201,76,.18);color:var(--yellow);}

  /* live reading tiles */
  .tiles{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;}
  .tile{background:rgba(255,255,255,.04);border:1px solid var(--border);border-radius:14px;
    padding:14px 6px;text-align:center;}
  .tile.active{background:rgba(79,216,196,.14);border-color:var(--teal);}
  .tile-label{font-size:.72em;color:var(--dim);}
  .tile-val{font-size:1.05em;font-weight:800;margin-top:5px;}

  /* output card */
  .output-row{display:flex;align-items:center;gap:14px;}
  .output-icon{width:46px;height:46px;border-radius:50%;background:rgba(255,255,255,.08);
    display:flex;align-items:center;justify-content:center;flex-shrink:0;}
  .output-icon svg{width:22px;height:22px;stroke:var(--teal);fill:none;stroke-width:2;stroke-linecap:round;}
  .output-value{font-size:1.35em;font-weight:800;}
  .output-label{font-size:.75em;color:var(--dim);margin-top:2px;}
  .output-badge{margin-left:auto;}

  footer{text-align:center;margin-top:22px;font-size:.72em;color:rgba(245,247,250,.35);}
</style>
</head>
<body>
<div class="wrap">

  <div class="header">
    <div class="brand">
      <h1>WallBot</h1>
      <p>Live dashboard</p>
    </div>
    <div class="header-right">
      <div class="status-pill"><span class="status-dot" id="statusDot"></span><span id="statusText">Live</span></div>
      <div class="avatar">&#129302;</div>
    </div>
  </div>

  <div id="stateBanner" class="state-banner">Loading&hellip;</div>

  <div class="top-grid">
    <div class="card">
      <div class="card-head"><h2>Distance sensors</h2><span class="hint">0&ndash;30 cm scale</span></div>
      <div class="bars">
        <div class="bar-col"><div class="bar-val" id="bvL">--</div><div class="bar" id="barL"></div><div class="bar-label">Left</div></div>
        <div class="bar-col"><div class="bar-val" id="bvF">--</div><div class="bar" id="barF"></div><div class="bar-label">Front</div></div>
        <div class="bar-col"><div class="bar-val" id="bvR">--</div><div class="bar" id="barR"></div><div class="bar-label">Right</div></div>
      </div>
    </div>

    <div class="stat-stack">
      <div class="stat-card">
        <div class="stat-icon"><svg viewBox="0 0 24 24"><path d="M20 12a8 8 0 1 1-3-6.2"/><path d="M20 4v5h-5"/></svg></div>
        <div><div class="stat-value" id="statLS">--</div><div class="stat-label">Left motor (PWM)</div></div>
      </div>
      <div class="stat-card">
        <div class="stat-icon"><svg viewBox="0 0 24 24"><path d="M4 12a8 8 0 1 0 3-6.2"/><path d="M4 4v5h5"/></svg></div>
        <div><div class="stat-value" id="statRS">--</div><div class="stat-label">Right motor (PWM)</div></div>
      </div>
      <div class="stat-card">
        <div class="stat-icon"><svg viewBox="0 0 24 24"><circle cx="12" cy="12" r="8"/><circle cx="12" cy="12" r="2.4"/></svg></div>
        <div><div class="stat-value" id="statErr">--</div><div class="stat-label">PID error</div></div>
      </div>
    </div>

    <div class="card">
      <div class="card-head"><h2>Overview</h2><span class="hint">Steering balance</span></div>
      <div class="donut-wrap">
        <svg viewBox="0 0 120 120">
          <circle class="donut-track" cx="60" cy="60" r="50"/>
          <circle id="donutA" class="donut-seg" cx="60" cy="60" r="50" transform="rotate(-90 60 60)" style="stroke:var(--blue)"/>
          <circle id="donutB" class="donut-seg" cx="60" cy="60" r="50" transform="rotate(-90 60 60)" style="stroke:var(--yellow)"/>
        </svg>
        <div class="donut-center"><div class="donut-pct" id="donutPct">--</div><div class="donut-sub" id="donutSub">--</div></div>
      </div>
      <div class="overview-list">
        <div class="o-row"><span class="o-dot" style="background:var(--teal)"></span>
          <div class="o-body"><div class="o-label">Mode</div><div class="o-value" id="ovMode">--</div></div></div>
        <div class="o-row"><span class="o-dot" style="background:var(--yellow)"></span>
          <div class="o-body"><div class="o-label">Front clearance</div><div class="o-value" id="ovFront">--</div>
          <div class="o-sub" id="ovFrontStatus">--</div></div></div>
        <div class="o-row"><span class="o-dot" style="background:var(--blue)"></span>
          <div class="o-body"><div class="o-label">PID correction</div><div class="o-value" id="ovErr">--</div></div></div>
      </div>
    </div>
  </div>

  <div class="bottom-grid">
    <div class="card">
      <div class="card-head"><h2>Wheel health</h2><span class="hint">AS5600 encoders</span></div>

      <div class="health-row">
        <div class="health-ring" id="ringL">--</div>
        <div class="health-info">
          <div class="health-title">Left wheel</div>
          <div class="health-sub" id="healthL">--</div>
        </div>
        <span class="badge" id="badgeL">--</span>
      </div>

      <div class="health-row">
        <div class="health-ring" id="ringR">--</div>
        <div class="health-info">
          <div class="health-title">Right wheel</div>
          <div class="health-sub" id="healthR">--</div>
        </div>
        <span class="badge" id="badgeR">--</span>
      </div>

      <div class="health-row">
        <div class="health-ring" id="ringF">--</div>
        <div class="health-info">
          <div class="health-title">Front clearance</div>
          <div class="health-sub" id="healthF">--</div>
        </div>
        <span class="badge" id="badgeF">--</span>
      </div>
    </div>

    <div>
      <div class="card" style="margin-bottom:16px;">
        <div class="card-head"><h2>Live readings</h2><span class="hint">cm</span></div>
        <div class="tiles">
          <div class="tile" id="tileL"><div class="tile-label">Left</div><div class="tile-val" id="tvL">--</div></div>
          <div class="tile" id="tileF"><div class="tile-label">Front</div><div class="tile-val" id="tvF">--</div></div>
          <div class="tile" id="tileR"><div class="tile-label">Right</div><div class="tile-val" id="tvR">--</div></div>
        </div>
      </div>

      <div class="card">
        <div class="card-head"><h2>Output</h2><span class="hint">Since power-on</span></div>
        <div class="output-row">
          <div class="output-icon"><svg viewBox="0 0 24 24"><path d="M3 12h4l3-8 4 16 3-8h4"/></svg></div>
          <div>
            <div class="output-value" id="outDist">--</div>
            <div class="output-label">Distance traveled</div>
          </div>
          <span class="badge output-badge" id="outBadge">--</span>
        </div>
      </div>
    </div>
  </div>

  <footer>Auto-refreshing every 400&nbsp;ms &middot; served from the ESP32's own WiFi access point</footer>
</div>

<script>
const CIRC = 2 * Math.PI * 50;
let missCount = 0;

function clamp(v,a,b){ return Math.max(a, Math.min(b, v)); }

async function poll(){
  try{
    const r = await fetch('/data');
    const d = await r.json();
    missCount = 0;
    document.getElementById('statusDot').className = 'status-dot';
    document.getElementById('statusText').textContent = 'Live';

    // state banner
    const banner = document.getElementById('stateBanner');
    banner.textContent = d.state;
    banner.className = 'state-banner ' + d.stateClass;

    // distance bars (scaled 0-30cm)
    const maxCm = 30;
    const setBar = (barId, valId, val, alert) => {
      document.getElementById(barId).style.height = clamp((val/maxCm)*100, 3, 100) + '%';
      document.getElementById(barId).className = 'bar' + (alert ? ' alert' : '');
      document.getElementById(valId).textContent = val.toFixed(1);
    };
    const frontAlert = d.frontDist < d.frontStopCm * 1.5;
    setBar('barL','bvL', d.leftDist, false);
    setBar('barF','bvF', d.frontDist, frontAlert);
    setBar('barR','bvR', d.rightDist, false);

    // stat stack
    document.getElementById('statLS').textContent = d.leftSpeed;
    document.getElementById('statRS').textContent = d.rightSpeed;
    document.getElementById('statErr').textContent = d.error.toFixed(2);

    // donut: steering balance between |leftSpeed| and |rightSpeed|
    const absL = Math.abs(d.leftSpeed), absR = Math.abs(d.rightSpeed);
    const total = absL + absR;
    const leftPct = total > 0 ? (absL/total)*100 : 50;
    const rightPct = 100 - leftPct;
    const lenA = CIRC * (leftPct/100);
    const lenB = CIRC * (rightPct/100);
    const donutA = document.getElementById('donutA');
    const donutB = document.getElementById('donutB');
    donutA.setAttribute('stroke-dasharray', lenA + ' ' + (CIRC-lenA));
    donutA.setAttribute('stroke-dashoffset', '0');
    donutB.setAttribute('stroke-dasharray', lenB + ' ' + (CIRC-lenB));
    donutB.setAttribute('stroke-dashoffset', -lenA);
    const dominant = leftPct >= rightPct ? 'Left' : 'Right';
    const diff = Math.abs(leftPct - rightPct);
    document.getElementById('donutPct').textContent = Math.round(Math.max(leftPct,rightPct)) + '%';
    document.getElementById('donutSub').textContent = diff < 6 ? 'Balanced steering' : dominant + ' wheel working harder';

    // overview list
    document.getElementById('ovMode').textContent = d.mode;
    document.getElementById('ovFront').textContent = d.frontDist.toFixed(1) + ' cm';
    const fOk = d.frontDist >= d.frontStopCm * 1.5;
    const fSub = document.getElementById('ovFrontStatus');
    fSub.textContent = fOk ? 'Clear' : 'Caution';
    fSub.style.color = fOk ? 'var(--green)' : 'var(--yellow)';
    document.getElementById('ovErr').textContent = d.error.toFixed(2);

    // wheel health
    const setRing = (ringId, ok) => {
      const el = document.getElementById(ringId);
      el.textContent = ok ? String.fromCharCode(10003) : String.fromCharCode(10005);
      el.className = 'health-ring' + (ok ? '' : ' err');
    };
    setRing('ringL', d.encLOk);
    setRing('ringR', d.encROk);
    document.getElementById('healthL').textContent = d.angleL.toFixed(1) + '\u00b0 \u00b7 ' + d.revL.toFixed(2) + ' rev \u00b7 ' + d.rpmL.toFixed(1) + ' RPM';
    document.getElementById('healthR').textContent = d.angleR.toFixed(1) + '\u00b0 \u00b7 ' + d.revR.toFixed(2) + ' rev \u00b7 ' + d.rpmR.toFixed(1) + ' RPM';
    const badgeL = document.getElementById('badgeL');
    badgeL.textContent = d.encLOk ? 'OK' : 'ERROR';
    badgeL.className = 'badge ' + (d.encLOk ? 'ok' : 'err');
    const badgeR = document.getElementById('badgeR');
    badgeR.textContent = d.encROk ? 'OK' : 'ERROR';
    badgeR.className = 'badge ' + (d.encROk ? 'ok' : 'err');

    setRing('ringF', fOk);
    document.getElementById('healthF').textContent = d.frontDist.toFixed(1) + ' cm to nearest obstacle';
    const badgeF = document.getElementById('badgeF');
    badgeF.textContent = fOk ? 'Clear' : 'Close!';
    badgeF.className = 'badge ' + (fOk ? 'ok' : 'warn');

    // live reading tiles - highlight the closest sensor
    const vals = {L:d.leftDist, F:d.frontDist, R:d.rightDist};
    const minKey = Object.keys(vals).reduce((a,b)=> vals[a] < vals[b] ? a : b);
    ['L','F','R'].forEach(k => {
      document.getElementById('tv'+k).textContent = vals[k].toFixed(1);
      document.getElementById('tile'+k).className = 'tile' + (k === minKey ? ' active' : '');
    });

    // output card
    document.getElementById('outDist').textContent = d.distanceM.toFixed(2) + ' m';
    const outBadge = document.getElementById('outBadge');
    outBadge.textContent = d.isMoving ? 'Moving' : 'Idle';
    outBadge.className = 'badge output-badge ' + (d.isMoving ? 'ok' : 'warn');

  }catch(e){
    missCount++;
    if (missCount >= 3) {
      document.getElementById('statusDot').className = 'status-dot lost';
      document.getElementById('statusText').textContent = 'Reconnecting\u2026';
    }
  }
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
  json += "\"rpmR\":"         + String(status.rpmR, 2) + ",";
  json += "\"distanceM\":"    + String(status.distanceTraveledM, 3) + ",";
  json += "\"isMoving\":"     + String(status.isMoving ? "true" : "false") + ",";
  json += "\"frontStopCm\":"  + String(FRONT_STOP_CM, 1);
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
    serverDelay(150);
    // Turn toward the side with more room
    if (rightDist > leftDist) {
      setMotors(150, -150);  // pivot right
      status.leftSpeed = 150; status.rightSpeed = -150;
    } else {
      setMotors(-150, 150);  // pivot left
      status.leftSpeed = -150; status.rightSpeed = 150;
    }
    serverDelay(300);
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

  float wheelCircumferenceCm = 3.14159265 * WHEEL_DIAMETER_CM;
  float avgRevs = (fabs((float)encL.totalCounts) + fabs((float)encR.totalCounts)) / 2.0 / COUNTS_PER_REV;
  status.distanceTraveledM = (avgRevs * wheelCircumferenceCm) / 100.0;
  status.isMoving          = (fabs(rpmL) > 2.0 || fabs(rpmR) > 2.0);

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
