#include "Sensors.h"

static float pingRaw(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long dur = pulseIn(echo, HIGH, ECHO_TIMEOUT_US);
  if (dur == 0) return 400.0f; // timeout -> treat as clear/open
  return dur * 0.01715f;
}

static float median3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

static float pingMedian(int trig, int echo) {
  float s1 = pingRaw(trig, echo); delayMicroseconds(300);
  float s2 = pingRaw(trig, echo); delayMicroseconds(300);
  float s3 = pingRaw(trig, echo);
  return median3(s1, s2, s3);
}

void Sensors::init() {
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT,  OUTPUT); pinMode(ECHO_LEFT,  INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);
  DBG_PRINTLN("[SENSORS] initialized");
}

SensorReading Sensors::readAll() {
  SensorReading r;
  r.front = pingMedian(TRIG_FRONT, ECHO_FRONT); delayMicroseconds(300);
  r.left  = pingMedian(TRIG_LEFT,  ECHO_LEFT);  delayMicroseconds(300);
  r.right = pingMedian(TRIG_RIGHT, ECHO_RIGHT);
  r.wallFront = r.front < FRONT_WALL_CM;
  r.wallLeft  = r.left  < WALL_CM;
  r.wallRight = r.right < WALL_CM;
  DBG_PRINTF("[SENSORS] F=%.1f L=%.1f R=%.1f | wallF=%d wallL=%d wallR=%d\n",
             r.front, r.left, r.right, r.wallFront, r.wallLeft, r.wallRight);
  return r;
}

float Sensors::readFrontFast() {
  return pingRaw(TRIG_FRONT, ECHO_FRONT);
}
