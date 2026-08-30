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

// ---------------- Setup ----------------
void setup() {
  pinMode(PIN_TRIG_LEFT, OUTPUT);  pinMode(PIN_ECHO_LEFT, INPUT);
  pinMode(PIN_TRIG_RIGHT, OUTPUT); pinMode(PIN_ECHO_RIGHT, INPUT);
  pinMode(PIN_TRIG_FRONT, OUTPUT); pinMode(PIN_ECHO_FRONT, INPUT);

  pinMode(PIN_STBY, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT); pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT); pinMode(PIN_PWMB, OUTPUT);

  digitalWrite(PIN_STBY, HIGH); // enable the driver (LOW = low-power standby, motors off)

  Serial.begin(115200);
  lastTime = millis();
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

// ---------------- Main loop ----------------
void loop() {
  float leftDist  = readDistanceCM(PIN_TRIG_LEFT,  PIN_ECHO_LEFT);
  float rightDist = readDistanceCM(PIN_TRIG_RIGHT, PIN_ECHO_RIGHT);
  float frontDist = readDistanceCM(PIN_TRIG_FRONT, PIN_ECHO_FRONT);

  // --- Dead-end / obstacle handling ---
  if (frontDist < FRONT_STOP_CM) {
    stopMotors();
    delay(150);
    // Turn toward the side with more room
    if (rightDist > leftDist) {
      setMotors(150, -150);  // pivot right
    } else {
      setMotors(-150, 150);  // pivot left
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

  // --- Debug ---
  Serial.print("L:"); Serial.print(leftDist);
  Serial.print(" R:"); Serial.print(rightDist);
  Serial.print(" F:"); Serial.print(frontDist);
  Serial.print(" Mode:"); Serial.print(rightWallPresent ? "CENTER" : "LEFT-ONLY");
  Serial.print(" err:"); Serial.print(error);
  Serial.print(" Lspd:"); Serial.print(leftSpeed);
  Serial.print(" Rspd:"); Serial.println(rightSpeed);

  delay(20); // small loop delay for sensor settling
}
