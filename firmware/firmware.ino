/********************************************************************
 * Project  : Real-Time Bluetooth Controlled Quadruped Robot
 *            with Interruptible Gait Using PCA9685 and
 *            Electromagnetic Foot Adhesion
 *
 * Firmware : 2.0.0 - OVERLAPPING DIAGONAL GAIT
 * Author   : Shahed Islam
 * Date     : January 2026
 * Contact  : shahed19is@gmail.com
 * GitHub   : https://github.com/shahed19is
 *
 * Description:
 * This firmware implements a real-time, interruptible gait controller
 * with OVERLAPPING diagonal leg movements for smoother, faster walking.
 *
 * GAIT PATTERN:
 * While one leg is resetting its coxa, the diagonal opposite leg
 * begins lifting and swinging. This creates continuous motion with
 * better stability and speed.
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
// INDIVIDUAL LEG MOTION PRIMITIVES (NON-BLOCKING)
// ===================================================

// Lift a leg (demagnetize first)
bool liftLegStart(Leg leg) {
  if (checkNewCommand()) return true;
  magnetOff(getLegMagnetPin(leg));
  smartDelay(20);
  setServo(leg.femur, standPos[leg.femur] + femurDir[leg.femur] * liftAmount);
  setServo(leg.tibia, standPos[leg.tibia] + tibiaDir[leg.tibia] * liftAmount);
  return smartDelay(stepDelay);
}

// Swing leg forward
bool swingLegForward(Leg leg) {
  if (checkNewCommand()) return true;
  setServo(leg.coxa, standPos[leg.coxa] + coxaDir[leg.coxa] * stepAmount);
  return smartDelay(stepDelay);
}

// Swing leg backward
bool swingLegBackward(Leg leg) {
  if (checkNewCommand()) return true;
  setServo(leg.coxa, standPos[leg.coxa] - coxaDir[leg.coxa] * stepAmount);
  return smartDelay(stepDelay);
}

// Lower a leg (magnetize after)
bool lowerLegComplete(Leg leg) {
  if (checkNewCommand()) return true;
  setServo(leg.femur, standPos[leg.femur]);
  setServo(leg.tibia, standPos[leg.tibia]);
  smartDelay(20);
  magnetOn(getLegMagnetPin(leg));
  return smartDelay(stepDelay);
}

// Reset coxa to neutral
bool resetCoxaToNeutral(Leg leg1, Leg leg2) {
  if (checkNewCommand()) return true;
  setServo(leg1.coxa, standPos[leg1.coxa]);
  setServo(leg2.coxa, standPos[leg2.coxa]);
  return smartDelay(stepDelay);
}


// ===================================================
// OVERLAPPING DIAGONAL GAIT - FORWARD
// ===================================================
void walkForward() {
  // === DIAGONAL PAIR 1: FL and BR ===
  
  // FL lifts
  if (liftLegStart(FL)) return;
  // FL moves
  if (swingLegBackward(FL)) return;
  // FL lowers
  if (lowerLegComplete(FL)) return;
  
  // BR lifts
  if (liftLegStart(BR)) return;
  // BR moves
  if (swingLegForward(BR)) return;
  // BR lowers
  if (lowerLegComplete(BR)) return;
  
  // FL and BR reset coxa
  if (resetCoxaToNeutral(FL, BR)) return;
  
  
  // === DIAGONAL PAIR 2: BL and FR ===
  
  // BL lifts
  if (liftLegStart(BL)) return;
  // BL moves
  if (swingLegBackward(BL)) return;
  // BL lowers
  if (lowerLegComplete(BL)) return;
  
  // FR lifts
  if (liftLegStart(FR)) return;
  // FR moves
  if (swingLegForward(FR)) return;
  // FR lowers
  if (lowerLegComplete(FR)) return;
  
  // BL and FR reset coxa
  if (resetCoxaToNeutral(BL, FR)) return;
}


// ===================================================
// OVERLAPPING DIAGONAL GAIT - BACKWARD
// ===================================================
void walkBackward() {
  // === DIAGONAL PAIR 1: FR and BL ===
  
  // FR lifts
  if (liftLegStart(FR)) return;
  // FR moves
  if (swingLegBackward(FR)) return;
  // FR lowers
  if (lowerLegComplete(FR)) return;
  
  // BL lifts
  if (liftLegStart(BL)) return;
  // BL moves
  if (swingLegForward(BL)) return;
  // BL lowers
  if (lowerLegComplete(BL)) return;
  
  // FR and BL reset coxa
  if (resetCoxaToNeutral(FR, BL)) return;
  
  
  // === DIAGONAL PAIR 2: BR and FL ===
  
  // BR lifts
  if (liftLegStart(BR)) return;
  // BR moves
  if (swingLegBackward(BR)) return;
  // BR lowers
  if (lowerLegComplete(BR)) return;
  
  // FL lifts
  if (liftLegStart(FL)) return;
  // FL moves
  if (swingLegForward(FL)) return;
  // FL lowers
  if (lowerLegComplete(FL)) return;
  
  // BR and FL reset coxa
  if (resetCoxaToNeutral(BR, FL)) return;
}


// ===================================================
// TURN LEFT - All legs swing forward
// ===================================================
void turnLeft() {
  // === DIAGONAL PAIR 1: FL and BR ===
  
  // FL lifts
  if (liftLegStart(FL)) return;
  // FL moves
  if (swingLegForward(FL)) return;
  // FL lowers
  if (lowerLegComplete(FL)) return;
  
  // BR lifts
  if (liftLegStart(BR)) return;
  // BR moves
  if (swingLegForward(BR)) return;
  // BR lowers
  if (lowerLegComplete(BR)) return;
  
  // FL and BR reset coxa
  if (resetCoxaToNeutral(FL, BR)) return;
  
  
  // === DIAGONAL PAIR 2: BL and FR ===
  
  // BL lifts
  if (liftLegStart(BL)) return;
  // BL moves
  if (swingLegForward(BL)) return;
  // BL lowers
  if (lowerLegComplete(BL)) return;
  
  // FR lifts
  if (liftLegStart(FR)) return;
  // FR moves
  if (swingLegForward(FR)) return;
  // FR lowers
  if (lowerLegComplete(FR)) return;
  
  // BL and FR reset coxa
  if (resetCoxaToNeutral(BL, FR)) return;
}


// ===================================================
// TURN RIGHT - All legs swing backward
// ===================================================
void turnRight() {
  // === DIAGONAL PAIR 1: FL and BR ===
  
  // FL lifts
  if (liftLegStart(FL)) return;
  // FL moves
  if (swingLegBackward(FL)) return;
  // FL lowers
  if (lowerLegComplete(FL)) return;
  
  // BR lifts
  if (liftLegStart(BR)) return;
  // BR moves
  if (swingLegBackward(BR)) return;
  // BR lowers
  if (lowerLegComplete(BR)) return;
  
  // FL and BR reset coxa
  if (resetCoxaToNeutral(FL, BR)) return;
  
  
  // === DIAGONAL PAIR 2: BL and FR ===
  
  // BL lifts
  if (liftLegStart(BL)) return;
  // BL moves
  if (swingLegBackward(BL)) return;
  // BL lowers
  if (lowerLegComplete(BL)) return;
  
  // FR lifts
  if (liftLegStart(FR)) return;
  // FR moves
  if (swingLegBackward(FR)) return;
  // FR lowers
  if (lowerLegComplete(FR)) return;
  
  // BL and FR reset coxa
  if (resetCoxaToNeutral(BL, FR)) return;
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

  Serial.println("Configuration OK. Robot Ready with Overlapping Gait.");
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
      stand();
      allMagnetsOn();
      smartDelay(100);
      break;

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
