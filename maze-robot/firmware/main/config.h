// =============================================================================
//  config.h  —  Central configuration for the maze-solving robot firmware
//  All hardware pins, network settings and tunable constants live here so
//  the rest of the firmware never hard-codes a number.
// =============================================================================
#pragma once
#include <Arduino.h>

// ── Debug ─────────────────────────────────────────────────────────────────
#define DEBUG 1
#if DEBUG
  #define DBG_PRINT(...)    Serial.print(__VA_ARGS__)
  #define DBG_PRINTLN(...)  Serial.println(__VA_ARGS__)
  #define DBG_PRINTF(...)   Serial.printf(__VA_ARGS__)
#else
  #define DBG_PRINT(...)
  #define DBG_PRINTLN(...)
  #define DBG_PRINTF(...)
#endif

// ── WiFi / Server ─────────────────────────────────────────────────────────
#define WIFI_SSID        "HOME"
#define WIFI_PASSWORD    "9904574199"
#define WS_HOST          "192.168.1.44"   // backend server IP
#define WS_PORT          8080              // same port as backend REST/Socket.IO server (see backend/server.js)
#define WS_PATH          "/robot"          // raw WS bridge path (see backend/websocket/robotBridge.js)
#define ROBOT_ID         "robot-01"
#define ROBOT_AUTH_TOKEN "U_GOT_A_PROBLEM_CHANGE_UR_FUCKING_CAR"  // shared secret, checked by backend

#define WIFI_RECONNECT_INTERVAL_MS   5000
#define WS_RECONNECT_INTERVAL_MS     3000
#define TELEMETRY_INTERVAL_MS        100   // spec: update every 100ms

// ── Motor Driver (TB6612FNG) ──────────────────────────────────────────────
#define PWMA 18
#define AIN1 25
#define AIN2 26
#define PWMB 19
#define BIN1 27
#define BIN2 14
#define STBY 13

#define MOTOR_A_INVERT 0
#define MOTOR_B_INVERT 1

// ── Ultrasonic Sensors (HC-SR04) ──────────────────────────────────────────
#define TRIG_FRONT 21
#define ECHO_FRONT 23
#define TRIG_LEFT  4
#define ECHO_LEFT  22
#define TRIG_RIGHT 15
#define ECHO_RIGHT 5

// ── Buzzer ────────────────────────────────────────────────────────────────
#define BUZZER_PIN 32
// Optional status LED (future-ready; comment out if not wired)
#define STATUS_LED_PIN 33

#define ECHO_TIMEOUT_US 25000UL

// ── Sensing thresholds ────────────────────────────────────────────────────
#define WALL_CM         18.0f   // side reading below this = wall present
#define STOP_CM         10.0f   // front safety stop distance
#define FRONT_WALL_CM   18.0f   // front reading below this (at rest) = wall present
#define OPEN_CONFIRM_COUNT 2

// ── Speeds ────────────────────────────────────────────────────────────────
#define BASE_SPEED     190
#define TURN_SPEED     150
#define NUDGE_SPEED    150
#define REVERSE_SPEED  150

// ── Wall-following correction (used mid-cell, forward motion) ───────────
#define CORR_KP          6.0f
#define CORR_MAX         50
#define CORR_DEADBAND_CM 1.0f
#define TARGET_LEFT_CM   9.0f

// ── Timing (CALIBRATE ON YOUR ROBOT) ─────────────────────────────────────
// These define the non-blocking motion-primitive durations, in ms.
#define NUDGE_MS        220   // creep so axle clears a corner before pivoting
#define TURN_MS         430   // time to pivot ~90 degrees at TURN_SPEED
#define UTURN_MS        860   // time to pivot ~180 degrees
#define SETTLE_MS        40   // brief pause between motion phases
#define CELL_TRAVEL_MS  700   // time to drive the length of one grid cell at BASE_SPEED
#define REVERSE_MS      300

// ── Maze grid ─────────────────────────────────────────────────────────────
#define GRID_SIZE 16     // 16x16 max maze (classic micromouse size, adjust freely)
#define MAX_STACK 256    // DFS backtrack stack depth (>= GRID_SIZE*GRID_SIZE)

// Goal cells: ultrasonic sensors cannot visually identify a "goal marker",
// so the goal is defined as a configurable target coordinate (classic
// micromouse convention: the centre of the maze). This can be overridden
// live from the dashboard via a SET_GOAL command (see WebSocketComm).
#define DEFAULT_GOAL_X (GRID_SIZE / 2)
#define DEFAULT_GOAL_Y (GRID_SIZE / 2)
#define START_X 0
#define START_Y 0
