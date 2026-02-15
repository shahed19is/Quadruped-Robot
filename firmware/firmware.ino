/********************************************************************
 * Project  : Real-Time Bluetooth Controlled Quadruped Robot
 *            with Interruptible Gait Using PCA9685 and
 *            Electromagnetic Foot Adhesion
 *
 * Firmware : 3.0.0 - SIMULTANEOUS MULTI-SERVO GAIT
 * Author   : Shahed Islam
 * Date     : February 2026
 * Contact  : shahed19is@gmail.com
 * GitHub   : https://github.com/shahed19is
 *
 * Description:
 * This firmware implements a real-time, interruptible gait controller
 * with SIMULTANEOUS servo movement for smooth, coordinated motion.
 * Multiple servos move together during each leg operation,
 * resulting in faster and more fluid walking patterns.
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

// Move a leg: lift → swing (dir: 11=forward, 10=backward) → lower
bool moveLeg(Leg leg, int dir) {
  if (checkNewCommand()) return true;
  
  magnetOff(getLegMagnetPin(leg));
  delay(magDelay);
  
  // Lift Leg
  setServo(leg.femur, standPos[leg.femur] + femurDir[leg.femur] * liftAmount);

  // dir means direction. Forward = 11, Backward = 10
  if(dir == 11) {
    setServo(leg.coxa, standPos[leg.coxa] + coxaDir[leg.coxa] * stepAmount);
  }
  else if (dir == 10) {
    setServo(leg.coxa, standPos[leg.coxa] - coxaDir[leg.coxa] * stepAmount);
  }
  smartDelay(stepDelay);
  
  // Lower Leg
  setServo(leg.femur, standPos[leg.femur]);
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
  
  
  // Reset current pair's coxa
  setServo(movLeg1.coxa, standPos[movLeg1.coxa]);
  setServo(movLeg2.coxa, standPos[movLeg2.coxa]);
  
  
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
  if (moveLeg(FL, 10)) return;
  
  // BR: lift → swing forward → lower
  if (moveLeg(BR, 11)) return;
  
  // === WEIGHT TRANSFER & RESET FL-BR ===
  if (resetCoxa(FL, BR, BL, FR)) return;
  
  // === DIAGONAL PAIR 2: BL and FR ===
  
  // BL: lift → swing backward → lower
  if (moveLeg(BL, 10)) return;
  
  // FR: lift → swing forward → lower
  if (moveLeg(FR, 11)) return;
  
  // === WEIGHT TRANSFER & RESET BL-FR ===
  if (resetCoxa(FR, BL, BR, FL)) return;
}


// ===================================================
// OVERLAPPING WEIGHT TRANSFER GAIT - BACKWARD
// ===================================================
void walkBackward() {
  // === DIAGONAL PAIR 1: FR and BL ===
  
  // FR: lift → swing forward → lower
  if (moveLeg(FR, 11)) return;
  
  // BL: lift → swing backward → lower
  if (moveLeg(BL, 10)) return;
  
  // === WEIGHT TRANSFER & RESET FR-BL ===
  if (resetCoxa(FR, BL, BR, FL)) return;
  
  // === DIAGONAL PAIR 2: BR and FL ===
  
  // BR: lift → swing forward → lower
  if (moveLeg(BR, 11)) return;
  
  // FL: lift → swing backward → lower
  if (moveLeg(FL, 10)) return;
  
  // === WEIGHT TRANSFER & RESET BR-FL ===
  if (resetCoxa(FL, BR, BL, FR)) return;
}


// ===================================================
// TURN LEFT - All legs swing forward
// ===================================================
void turnLeft() {
  // === DIAGONAL PAIR 1: FL and BR ===
  
  if (moveLeg(FL, 11)) return;
  
  if (moveLeg(BR, 11)) return;
  
  // Weight transfer for turn
  if (resetCoxa(FL, BR, BL, FR)) return;
  
  // === DIAGONAL PAIR 2: BL and FR ===
  
  if (moveLeg(BL, 11)) return;
  
  if (moveLeg(FR, 11)) return;
  
  if (resetCoxa(BL, FR, FL, BR)) return;
}


// ===================================================
// TURN RIGHT - All legs swing backward
// ===================================================
void turnRight() {
  // === DIAGONAL PAIR 1: FL and BR ===
  
  if (moveLeg(FL, 10)) return;
  
  if (moveLeg(BR, 10)) return;
  
  // Weight transfer for turn
  if (resetCoxa(FL, BR, BL, FR)) return;
  
  
  // === DIAGONAL PAIR 2: BL and FR ===
  
  if (moveLeg(BL, 10)) return;
  
  if (moveLeg(FR, 10)) return;
  
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

  Serial.println("Configuration OK. Robot Ready with Weight Transfer Gait V3.0.0");
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
