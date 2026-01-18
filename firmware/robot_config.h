#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/********************************************************************
 * File     : robot_config.h
 * Project  : Real-Time Bluetooth Controlled Quadruped Robot
 *            with Interruptible Gait Using PCA9685
 *
 * Firmware : 1.1.0
 * Author   : Shahed Islam
 * Date     : January 2026
 * Contact  : shahed19is@gmail.com
 * GitHub   : https://github.com/shahed19is
 *
 * Description:
 * This header file declares all hardware configuration variables,
 * calibration parameters, and safety validation functions used by
 * the quadruped robot firmware.
 *
 * The actual definitions of these parameters are located in:
 *    robot_config.cpp
 *
 * Separating declarations and definitions ensures:
 *  - Clean firmware structure
 *  - Safe calibration management
 *  - Easier long-term maintenance
 ********************************************************************/


// ================= SERVO SAFETY LIMITS =================
//
// These limits protect the servos from over-driving.
// Every active PWM value must stay within this range.
//
#define SERVO_MIN 200
#define SERVO_MAX 550


// ================= ELECTROMAGNET PIN DEFINITIONS =================
//
// LOW  → Magnetized  (ON)
// HIGH → Demagnetized (OFF)
//
// Each leg has one electromagnet for surface adhesion.
//
#define MAGNET_FR 2   // Front Right Leg Magnet
#define MAGNET_FL 3   // Front Left  Leg Magnet
#define MAGNET_BR 4   // Back  Right Leg Magnet
#define MAGNET_BL 5   // Back  Left  Leg Magnet


// ================= LEG STRUCT =================
//
// Each leg consists of three joints:
//
//   Coxa  → Horizontal rotation (Arm joint)
//   Femur → Vertical lift       (Middle joint)
//   Tibia → Foot movement       (Lower joint)
//
struct Leg {
  int coxa;
  int femur;
  int tibia;
};


// ================= CONFIGURATION DECLARATIONS =================
//
// All variables below are defined in robot_config.cpp.
// They describe the physical and mechanical configuration
// of the quadruped robot.
//

extern int standPos[16];

extern Leg FL;
extern Leg FR;
extern Leg BL;
extern Leg BR;

extern int femurDir[16];
extern int tibiaDir[16];
extern int coxaDir[16];

extern int liftAmount;
extern int stepAmount;
extern int stepDelay;


// ================= CONFIGURATION VALIDATION =================
//
// This function checks whether all calibration values are
// inside safe operating limits. If any invalid value is
// detected, firmware execution must be halted.
//
bool validateConfiguration();

#endif
