/*
  ============================================================================
  SENSOR TEST — Left / Front / Right HC-SR04 ultrasonics only
  ============================================================================
  Purpose: verify all three ultrasonic sensors are wired correctly and
  returning sane distances BEFORE running the full maze_solver_v2 state
  machine. No motors, no turning, no path memory — just continuous readings
  printed to Serial.

  Wiring matches maze_solver_v2.ino exactly, so you can test on the same
  robot without touching any pins.

  How to use:
    1. Upload this sketch.
    2. Open Serial Monitor (or Serial Plotter) at 115200 baud.
    3. Wave a hand / book in front of each sensor and confirm the numbers
       respond correctly and match roughly what you'd measure with a tape
       measure. "-1" means that sensor timed out (no echo / open air beyond
       ~5 m / bad wiring).
  ============================================================================
*/

#include <Arduino.h>

// ---- HC-SR04 pins (same as maze_solver_v2.ino) ----
#define PIN_TRIG_LEFT     4
#define PIN_ECHO_LEFT    22
#define PIN_TRIG_FRONT   5
#define PIN_ECHO_FRONT   21
#define PIN_TRIG_RIGHT   16
#define PIN_ECHO_RIGHT   17

// Set to 1 if the Right sensor isn't physically wired yet — it will just
// mirror the Left sensor so you can still confirm Left + Front work.
#define RIGHT_SENSOR_MIRROR_LEFT   0

#define SAMPLES_PER_READ           3       // median-of-N, same as main code
#define SENSOR_TIMEOUT_US      30000       // ~5 m max range
#define READ_INTERVAL_MS         200       // how often to print a new set

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_TRIG_LEFT, OUTPUT);
  pinMode(PIN_ECHO_LEFT, INPUT);
  pinMode(PIN_TRIG_FRONT, OUTPUT);
  pinMode(PIN_ECHO_FRONT, INPUT);
  pinMode(PIN_TRIG_RIGHT, OUTPUT);
  pinMode(PIN_ECHO_RIGHT, INPUT);

  Serial.println(F("Sensor-only test starting..."));
  Serial.println(F("L(cm)\tF(cm)\tR(cm)"));
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  float leftCm  = readFilteredCM(PIN_TRIG_LEFT,  PIN_ECHO_LEFT);
  float frontCm = readFilteredCM(PIN_TRIG_FRONT, PIN_ECHO_FRONT);
  float rightCm =
#if RIGHT_SENSOR_MIRROR_LEFT
      leftCm;
#else
      readFilteredCM(PIN_TRIG_RIGHT, PIN_ECHO_RIGHT);
#endif

  Serial.print(leftCm, 1);
  Serial.print('\t');
  Serial.print(frontCm, 1);
  Serial.print('\t');
  Serial.println(rightCm, 1);

  delay(READ_INTERVAL_MS);
}

// ============================================================================
// SENSOR READING (identical logic to maze_solver_v2.ino, so results here
// are directly comparable to what the full state machine will see)
// ============================================================================

float readUltrasonicOnceCM(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, SENSOR_TIMEOUT_US);
  if (duration == 0) return -1;              // timeout / no echo
  return duration * 0.0343f / 2.0f;          // speed of sound -> cm
}

// Median-of-N filter — rejects single-sample ultrasonic glitches.
float readFilteredCM(uint8_t trigPin, uint8_t echoPin) {
  float samples[SAMPLES_PER_READ];
  uint8_t validCount = 0;

  for (uint8_t i = 0; i < SAMPLES_PER_READ; i++) {
    float d = readUltrasonicOnceCM(trigPin, echoPin);
    if (d > 0) samples[validCount++] = d;
    delayMicroseconds(500);
  }
  if (validCount == 0) return -1;

  for (uint8_t i = 1; i < validCount; i++) {
    float key = samples[i];
    int j = i - 1;
    while (j >= 0 && samples[j] > key) { samples[j + 1] = samples[j]; j--; }
    samples[j + 1] = key;
  }
  return samples[validCount / 2];
}
