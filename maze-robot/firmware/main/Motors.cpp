#include "Motors.h"

static void setMotorA(int speed) {
  int c = constrain(speed, -255, 255);
  int a = MOTOR_A_INVERT ? -c : c;
  digitalWrite(AIN1, a >= 0 ? LOW : HIGH);
  digitalWrite(AIN2, a >= 0 ? HIGH : LOW);
  ledcWrite(PWMA, abs(a));
}

static void setMotorB(int speed) {
  int c = constrain(speed, -255, 255);
  int b = MOTOR_B_INVERT ? -c : c;
  digitalWrite(BIN1, b >= 0 ? LOW : HIGH);
  digitalWrite(BIN2, b >= 0 ? HIGH : LOW);
  ledcWrite(PWMB, abs(b));
}

void Motors::init() {
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  ledcAttach(PWMA, 22000, 8);
  ledcAttach(PWMB, 22000, 8);
  DBG_PRINTLN("[MOTORS] initialized");
}

void Motors::stop()                      { setMotorA(0); setMotorB(0); }
void Motors::forward(int s)              { setMotorA(s); setMotorB(s); }
void Motors::reverse(int s)              { setMotorA(-s); setMotorB(-s); }
void Motors::differential(int l, int r)  { setMotorA(l); setMotorB(r); }
void Motors::pivotLeft(int s)            { setMotorA(-s); setMotorB(s); }
void Motors::pivotRight(int s)           { setMotorA(s); setMotorB(-s); }
