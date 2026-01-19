/********************************************************************
 * Project  : Real-Time Bluetooth Controlled Quadruped Robot
 *            with Interruptible Gait Using PCA9685 and
 *            Electromagnetic Foot Adhesion
 *
 * Firmware : 2.0.0 - OVERLAPPING WEIGHT TRANSFER GAIT
 * Author   : Shahed Islam
 * Date     : January 2026
 * Contact  : shahed19is@gmail.com
 * GitHub   : https://github.com/shahed19is
 *
 * Description:
 * This firmware implements a real-time, interruptible gait controller
 * with OVERLAPPING weight transfer between diagonal pairs.
 *
 * GAIT PATTERN WITH WEIGHT TRANSFER:
 * While one diagonal pair resets its coxa, the next pair prepares
 * to move by releasing magnets. The current pair slightly lowers
 * to transfer weight, then returns to normal height. This creates
 * smooth, continuous motion with better stability.
 *
 * All calibration constants, servo directions, mechanical mappings,
 * and electromagnet pin assignments are defined in:
 *    robot_config.h
 *    robot_config.cpp
 ********************************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "robot_config.h"

Adafruit_PWMServoDriver pwm(0x40);

// ================= GLOBAL STATE =================
char currentCmd = 'V';
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
// LOW  = Magnetized (ON)
// HIGH = Demagnetized (OFF)
// ===================================================
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
// SMART DELAY WITH COMMAND CHECK
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
  smartDelay(50);
}


// ===================================================
// INDIVIDUAL LEG MOTION PRIMITIVES
// ===================================================

// Lift a leg (demagnetize first)
bool liftLeg(Leg leg) {
  if (checkNewCommand()) return true;
  
  magnetOff(getLegMagnetPin(leg));
  delay(magDelay);
  
  setServo(leg.femur, standPos[leg.femur] + femurDir[leg.femur] * liftAmount);
  setServo(leg.tibia, standPos[leg.tibia] + tibiaDir[leg.tibia] * liftAmount);
  smartDelay(stepDelay);
  
  return false;
}

// Swing leg forward
bool swingForward(Leg leg) {
  if (checkNewCommand()) return true;
  
  setServo(leg.coxa, standPos[leg.coxa] + coxaDir[leg.coxa] * stepAmount);
  smartDelay(stepDelay);
  
  return false;
}

// Swing leg backward
bool swingBackward(Leg leg) {
  if (checkNewCommand()) return true;
  
  setServo(leg.coxa, standPos[leg.coxa] - coxaDir[leg.coxa] * stepAmount);
  smartDelay(stepDelay);
  
  return false;
}

// Lower a leg (magnetize after)
bool lowerLeg(Leg leg) {
  if (checkNewCommand()) return true;
  
  setServo(leg.femur, standPos[leg.femur]);
  setServo(leg.tibia, standPos[leg.tibia]);
  smartDelay(stepDelay);
  
  magnetOn(getLegMagnetPin(leg));
  delay(magDelay);
  
  return false;
}

// Reset coxa to neutral
bool resetCoxa(Leg movLeg1, Leg movLeg2, Leg relLeg1, Leg relLeg2) {
  if (checkNewCommand()) return true;
  
  smartDelay(magDelay);
  // Release next pair's magnets
  magnetOff(getLegMagnetPin(relLeg1));
  magnetOff(getLegMagnetPin(relLeg2));
  delay(magDelay);
  
  // Lower current pair slightly to transfer weight
  setServo(movLeg1.femur, standPos[movLeg1.femur] - femurDir[movLeg1.femur] * liftAmount * 2);
  setServo(movLeg2.femur, standPos[movLeg2.femur] - femurDir[movLeg2.femur] * liftAmount * 2);
  
  // Reset current pair's coxa
  setServo(movLeg1.coxa, standPos[movLeg1.coxa]);
  setServo(movLeg2.coxa, standPos[movLeg2.coxa]);
  
  // Raise current pair back to normal height
  setServo(movLeg1.femur, standPos[movLeg1.femur]);
  setServo(movLeg2.femur, standPos[movLeg2.femur]);
  delay(magDelay);
  
  // Re-engage next pair's magnets
  magnetOn(getLegMagnetPin(relLeg1));
  magnetOn(getLegMagnetPin(relLeg2));
  smartDelay(magDelay);
  
  return false;
}


// ===================================================
// OVERLAPPING WEIGHT TRANSFER GAIT - FORWARD
// ===================================================
void walkForward() {
  // === DIAGONAL PAIR 1: FL and BR ===
  
  // FL: lift → swing backward → lower
  if (liftLeg(FL)) return;
  if (swingBackward(FL)) return;
  if (lowerLeg(FL)) return;
  
  // BR: lift → swing forward → lower
  if (liftLeg(BR)) return;
  if (swingForward(BR)) return;
  if (lowerLeg(BR)) return;
  
  // === WEIGHT TRANSFER & RESET FL-BR ===
  if (resetCoxa(FL, BR, BL, FR)) return;
  
  // === DIAGONAL PAIR 2: BL and FR ===
  
  // BL: lift → swing backward → lower
  if (liftLeg(BL)) return;
  if (swingBackward(BL)) return;
  if (lowerLeg(BL)) return;
  
  // FR: lift → swing forward → lower
  if (liftLeg(FR)) return;
  if (swingForward(FR)) return;
  if (lowerLeg(FR)) return;
  
  // === WEIGHT TRANSFER & RESET BL-FR ===
  if (resetCoxa(FR, BL, BR, FL)) return;
}


// ===================================================
// OVERLAPPING WEIGHT TRANSFER GAIT - BACKWARD
// ===================================================
void walkBackward() {
  // === DIAGONAL PAIR 1: FR and BL ===
  
  // FR: lift → swing backward → lower
  if (liftLeg(FR)) return;
  if (swingBackward(FR)) return;
  if (lowerLeg(FR)) return;
  
  // BL: lift → swing forward → lower
  if (liftLeg(BL)) return;
  if (swingForward(BL)) return;
  if (lowerLeg(BL)) return;
  
  // === WEIGHT TRANSFER & RESET FR-BL ===
  if (resetCoxa(FR, BL, BR, FL)) return;
  
  // === DIAGONAL PAIR 2: BR and FL ===
  
  // BR: lift → swing backward → lower
  if (liftLeg(BR)) return;
  if (swingBackward(BR)) return;
  if (lowerLeg(BR)) return;
  
  // FL: lift → swing forward → lower
  if (liftLeg(FL)) return;
  if (swingForward(FL)) return;
  if (lowerLeg(FL)) return;
  
  // === WEIGHT TRANSFER & RESET BR-FL ===
  if (resetCoxa(FL, BR, BL, FR)) return;
}


// ===================================================
// TURN LEFT - All legs swing forward
// ===================================================
void turnLeft() {
  // === DIAGONAL PAIR 1: FL and BR ===
  
  if (liftLeg(FL)) return;
  if (swingForward(FL)) return;
  if (lowerLeg(FL)) return;
  
  if (liftLeg(BR)) return;
  if (swingForward(BR)) return;
  if (lowerLeg(BR)) return;
  
  // Weight transfer for turn
 if (resetCoxa(FL, BR, BL, FR)) return;
  
  // === DIAGONAL PAIR 2: BL and FR ===
  
  if (liftLeg(BL)) return;
  if (swingForward(BL)) return;
  if (lowerLeg(BL)) return;
  
  if (liftLeg(FR)) return;
  if (swingForward(FR)) return;
  if (lowerLeg(FR)) return;
  
  if (resetCoxa(BL, FR, FL, BR)) return;
}


// ===================================================
// TURN RIGHT - All legs swing backward
// ===================================================
void turnRight() {
  // === DIAGONAL PAIR 1: FL and BR ===
  
  if (liftLeg(FL)) return;
  if (swingBackward(FL)) return;
  if (lowerLeg(FL)) return;
  
  if (liftLeg(BR)) return;
  if (swingBackward(BR)) return;
  if (lowerLeg(BR)) return;
  
  // Weight transfer for turn
  if (resetCoxa(FL, BR, BL, FR)) return;
  
  
  // === DIAGONAL PAIR 2: BL and FR ===
  
  if (liftLeg(BL)) return;
  if (swingBackward(BL)) return;
  if (lowerLeg(BL)) return;
  
  if (liftLeg(FR)) return;
  if (swingBackward(FR)) return;
  if (lowerLeg(FR)) return;
  
  // Weight transfer for turn
  if (resetCoxa(BL, FR, FL, BR)) return;
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
    while (1);
  }

  pwm.begin();
  pwm.setPWMFreq(50);
  delay(1000);

  stand();
  allMagnetsOn();
  currentCmd = 'V';

  Serial.println("Configuration OK. Robot Ready with Weight Transfer Gait.");
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

    case 'V':
    case 'S':
      stand();
      allMagnetsOn();
      smartDelay(100);
      break;

    default:
      stand();
      allMagnetsOn();
      smartDelay(100);
      break;
  }
}
