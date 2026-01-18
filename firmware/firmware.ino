/********************************************************************
 * Project  : Real-Time Bluetooth Controlled Quadruped Robot
 *            with Interruptible Gait Using PCA9685 and
 *            Electromagnetic Foot Adhesion
 *
 * Firmware : 1.1.0
 * Author   : Shahed Islam
 * Date     : January 2026
 * Contact  : shahed19is@gmail.com
 * GitHub   : https://github.com/shahed19is
 *
 * Description:
 * This firmware implements a real-time, interruptible gait controller
 * for a quadruped robot equipped with electromagnetic feet. The robot
 * continuously monitors incoming Bluetooth/Serial commands and
 * immediately interrupts its current motion whenever a new command
 * is detected.
 *
 * The electromagnetic system allows each leg to magnetize or
 * demagnetize the ground surface to improve stability and traction
 * during movement, especially on metallic platforms.
 *
 * All calibration constants, servo directions, mechanical mappings,
 * and electromagnet pin assignments are defined in:
 *    robot_config.h
 *    robot_config.cpp
 *
 * This file only contains:
 *  - Motion logic
 *  - Command handling
 *  - Gait execution
 *  - Electromagnet control logic
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
// ELECTROMAGNET CONTROL
// ===================================================
//
// LOW  → Magnetized  (ON)
// HIGH → Demagnetized (OFF)
//
void magnetOn(int pin) {
  digitalWrite(pin, LOW);
}

void magnetOff(int pin) {
  digitalWrite(pin, HIGH);
}

void allMagnetsOn() {
  magnetOn(MAGNET_FL);
  magnetOn(MAGNET_FR);
  magnetOn(MAGNET_BL);
  magnetOn(MAGNET_BR);
}

void allMagnetsOff() {
  magnetOff(MAGNET_FL);
  magnetOff(MAGNET_FR);
  magnetOff(MAGNET_BL);
  magnetOff(MAGNET_BR);
}


// ===================================================
// MAP LEG TO ITS CORRESPONDING ELECTROMAGNET
// ===================================================
int getLegMagnetPin(Leg leg) {
  if (leg.coxa == FL.coxa) return MAGNET_FL;
  if (leg.coxa == FR.coxa) return MAGNET_FR;
  if (leg.coxa == BL.coxa) return MAGNET_BL;
  if (leg.coxa == BR.coxa) return MAGNET_BR;
  return -1;
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
// LEG MOTION PRIMITIVES (INTERRUPTIBLE + MAGNETIC)
// ===================================================
bool liftLeg(Leg leg) {
  if (checkNewCommand()) return true;

  // Release magnetic grip before lifting the leg
  magnetOff(getLegMagnetPin(leg));
  smartDelay(20);  // Allow relay/magnet to switch state

  setServo(leg.femur, standPos[leg.femur] + femurDir[leg.femur] * liftAmount);
  setServo(leg.tibia, standPos[leg.tibia] + tibiaDir[leg.tibia] * liftAmount);

  return smartDelay(stepDelay);
}

bool lowerLeg(Leg leg) {
  if (checkNewCommand()) return true;

  setServo(leg.femur, standPos[leg.femur]);
  setServo(leg.tibia, standPos[leg.tibia]);
  smartDelay(stepDelay);

  // Engage magnetic grip after placing the leg
  magnetOn(getLegMagnetPin(leg));
  return smartDelay(20);
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
  if (stepForward(BR)) return;
  if (stepBackward(BL)) return;
  if (stepForward(FR)) return;
}

void walkBackward() {
  if (stepBackward(FR)) return;
  if (stepForward(BL)) return;
  if (stepBackward(BR)) return;
  if (stepForward(FL)) return;
}

void turnLeft() {
  if (stepForward(FR)) return;
  if (stepForward(BL)) return;
  if (stepForward(FL)) return;
  if (stepForward(BR)) return;
}

void turnRight() {
  if (stepBackward(FR)) return;
  if (stepBackward(BL)) return;
  if (stepBackward(BR)) return;
  if (stepBackward(FL)) return;
}



// ===================================================
// SETUP
// ===================================================
void setup() {
  Serial.begin(9600);

  // Initialize electromagnet control pins
  pinMode(MAGNET_FL, OUTPUT);
  pinMode(MAGNET_FR, OUTPUT);
  pinMode(MAGNET_BL, OUTPUT);
  pinMode(MAGNET_BR, OUTPUT);

  if (!validateConfiguration()) {
    Serial.println("FATAL ERROR: Invalid servo configuration!");
    Serial.println("Check robot_config.cpp values.");
    while (1);   // Halt execution permanently
  }

  pwm.begin();
  pwm.setPWMFreq(50);
  delay(1000);

  // Initialize robot in stable standing posture
  stand();
  allMagnetsOn();
  currentCmd = 'V';

  Serial.println("Configuration OK. Robot Ready with Magnetic System.");
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
      allMagnetsOn();
      smartDelay(100);
      break;

    case 'S':   // Stop and return to stand
      stand();
      allMagnetsOn();
      smartDelay(100);
      break;

    default:    // Safety fallback
      stand();
      allMagnetsOn();
      smartDelay(100);
      break;
  }
}
