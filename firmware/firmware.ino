/********************************************************************
 * Project  : Real-Time Bluetooth Controlled Quadruped Robot
 *            with Interruptible Gait Using PCA9685
 *
 * Firmware : 1.0.0
 * Author   : Shahed Islam
 * Date     : January 2026
 * Contact  : shahed19is@gmail.com
 * GitHub   : https://github.com/shahed19is
 *
 * Description:
 * This firmware implements a real-time, interruptible gait controller
 * for a quadruped robot. The robot continuously monitors incoming
 * Bluetooth/Serial commands and immediately interrupts its current
 * motion whenever a new command is detected.
 *
 * All calibration constants, servo directions, and mechanical mappings
 * are defined in:
 *    robot_config.h
 *    robot_config.cpp
 *
 * This file only contains:
 *  - Motion logic
 *  - Command handling
 *  - Gait execution
 *  - Interruptible behavior
 ********************************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "robot_config.h"

Adafruit_PWMServoDriver pwm(0x40);

// ================= GLOBAL STATE =================
char currentCmd = 'V';      // Active command
unsigned long lastCmdTime = 0;

// ===================================================
// READ LATEST COMMAND (BUFFER CLEAN + CHANGE DETECTION)
// Keeps only the most recent valid character
// ===================================================
bool checkNewCommand() {
  if (Serial.available()) {
    char newCmd = '\0';

    // Flush buffer and keep last valid character
    while (Serial.available()) {
      char incoming = Serial.read();
      if (incoming != '\n' && incoming != '\r' && incoming != ' ') {
        newCmd = incoming;
      }
    }

    if (newCmd != '\0' && newCmd != currentCmd) {
      currentCmd = newCmd;
      lastCmdTime = millis();
      Serial.print("New Command Accepted: ");
      Serial.println(currentCmd);
      return true;
    }
  }
  return false;
}

// ===================================================
// LOW LEVEL SERVO CONTROL
// ===================================================
void setServo(int ch, int val) {
  val = constrain(val, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(ch, 0, val);
}

// ===================================================
// SMART DELAY
// Allows immediate interruption by new command
// ===================================================
bool smartDelay(int ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    if (checkNewCommand()) return true;
    delayMicroseconds(100);
  }
  return false;
}

// ===================================================
// STAND POSTURE
// ===================================================
void stand() {
  for (int i = 0; i < 16; i++) {
    if (standPos[i] != 0)
      setServo(i, standPos[i]);
  }
}

// ===================================================
// LEG MOTION PRIMITIVES (INTERRUPTIBLE)
// ===================================================
bool liftLeg(Leg leg) {
  if (checkNewCommand()) return true;
  setServo(leg.femur, standPos[leg.femur] + femurDir[leg.femur] * liftAmount);
  setServo(leg.tibia, standPos[leg.tibia] + tibiaDir[leg.tibia] * liftAmount);
  return smartDelay(stepDelay);
}

bool lowerLeg(Leg leg) {
  if (checkNewCommand()) return true;
  setServo(leg.femur, standPos[leg.femur]);
  setServo(leg.tibia, standPos[leg.tibia]);
  return smartDelay(stepDelay);
}

bool swingForward(Leg leg) {
  if (checkNewCommand()) return true;
  setServo(leg.coxa, standPos[leg.coxa] + coxaDir[leg.coxa] * stepAmount);
  return smartDelay(stepDelay);
}

bool swingBackward(Leg leg) {
  if (checkNewCommand()) return true;
  setServo(leg.coxa, standPos[leg.coxa] - coxaDir[leg.coxa] * stepAmount);
  return smartDelay(stepDelay);
}

bool resetCoxa(Leg leg) {
  if (checkNewCommand()) return true;
  setServo(leg.coxa, standPos[leg.coxa]);
  return smartDelay(stepDelay);
}

// ===================================================
// STEP SEQUENCES
// ===================================================
bool stepForward(Leg leg) {
  if (liftLeg(leg)) return true;
  if (swingForward(leg)) return true;
  if (lowerLeg(leg)) return true;
  if (resetCoxa(leg)) return true;
  return false;
}

bool stepBackward(Leg leg) {
  if (liftLeg(leg)) return true;
  if (swingBackward(leg)) return true;
  if (lowerLeg(leg)) return true;
  if (resetCoxa(leg)) return true;
  return false;
}

// ===================================================
// MOVEMENT FUNCTIONS (FULLY INTERRUPTIBLE)
// ===================================================
void walkForward() {
  if (stepBackward(FL)) return;
  if (stepBackward(BL)) return;
  if (stepForward(FR)) return;
  if (stepForward(BR)) return;
}

void walkBackward() {
  if (stepBackward(FR)) return;
  if (stepBackward(BR)) return;
  if (stepForward(FL)) return;
  if (stepForward(BL)) return;
}

void turnLeft() {
  if (stepForward(FR)) return;
  if (stepForward(BL)) return;
  if (stepForward(FL)) return;
  if (stepForward(BR)) return;
}

void turnRight() {
  if (stepBackward(FR)) return;
  if (stepBackward(BR)) return;
  if (stepBackward(FL)) return;
  if (stepBackward(BL)) return;
}

// ===================================================
// SETUP
// ===================================================
void setup() {
  Serial.begin(9600);

  if (!validateConfiguration()) {
    Serial.println("FATAL ERROR: Invalid servo configuration!");
    Serial.println("Check robot_config.cpp values.");
    while (1);   // Stop execution forever
  }

  pwm.begin();
  pwm.setPWMFreq(50);
  delay(1000);

  stand();
  currentCmd = 'V';

  Serial.println("Configuration OK. Robot Ready.");
}


// ===================================================
// MAIN LOOP
// ===================================================
void loop() {
  checkNewCommand();

  switch (currentCmd) {
    case 'F':
      walkForward();
      break;

    case 'B':
      walkBackward();
      break;

    case 'L':
      turnLeft();
      break;

    case 'R':
      turnRight();
      break;

    case 'V':   // Stand posture only
      stand();
      smartDelay(100);
      break;

    case 'S':   // Stop and return to stand
      stand();
      smartDelay(100);
      break;

    default:    // Safety fallback
      stand();
      smartDelay(100);
      break;
  }
}
