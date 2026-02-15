/************************************************************
 * File     : robot_config.cpp
 * Project  : Real-Time Bluetooth Controlled Quadruped Robot
 *            with Interruptible Gait Using PCA9685
 *
 * Firmware : 3.0.0
 * Author   : Shahed Islam
 * Date     : January 2026
 * Contact  : shahed19is@gmail.com
 * GitHub   : https://github.com/shahed19is
 *
 * Description:
 * This file contains all hardware calibration parameters and
 * mechanical configuration data for the quadruped robot with
 * electromagnetic feet support.
 *
 * WARNING:
 * Incorrect values in this file may damage servos, electromagnets,
 * or mechanical structures. Always keep all active PWM values
 * within:
 *
 *        SERVO_MIN (110)  ≤  PWM  ≤  SERVO_MAX (500)
 *
 * A value of 0 means the PCA9685 channel is unused.
 * Any invalid value will cause the firmware to halt at startup.
 ************************************************************/

#include "robot_config.h"


/* ================= STAND POSITIONS =================
 * Default standing posture for all servos.
 *
 * Index  : PCA9685 channel number (0–15)
 * Value  : PWM pulse width
 *
 * Rules:
 *   - SERVO_MIN ≤ Value ≤ SERVO_MAX
 *   - Value = 0  → Channel unused
 */
int standPos[16] = {
  300,  // Ch0  BR Tibia
  200,  // Ch1  BR Femur
  250,  // Ch2  BR Coxa
  0,    // Ch3  Unused

  300,  // Ch4  BL Tibia
  200,  // Ch5  BL Femur
  350,  // Ch6  BL Coxa
  0,    // Ch7  Unused

  350,  // Ch8  FR Coxa
  400,  // Ch9  FR Femur
  300,  // Ch10 FR Tibia
  0,    // Ch11 Unused

  250,  // Ch12 FL Coxa
  400,  // Ch13 FL Femur
  300,  // Ch14 FL Tibia
  0     // Ch15 Unused
};



/* ================= LEG DEFINITIONS =================
 * Each leg consists of three joints:
 *   Coxa  → Horizontal rotation (Arm)
 *   Femur → Vertical lift (Middle joint)
 *   Tibia → Foot movement (Lower joint)
 *
 * Format:
 *   { Coxa_Channel, Femur_Channel, Tibia_Channel }
 */
Leg BL = {6, 5, 4};     // Back Left Leg
Leg BR = {2, 1, 0};     // Back Right Leg
Leg FL = {12, 13, 14};  // Front Left Leg
Leg FR = {8, 9, 10};    // Front Right Leg


/* ================= SERVO DIRECTION POLARITY =================
 * Direction convention:
 *   +1 → Increasing PWM moves the joint UP / FORWARD
 *   -1 → Decreasing PWM moves the joint UP / FORWARD
 *    0 → Channel unused for that joint type
 *
 * These arrays define the mechanical orientation of every servo.
 * They must match the physical mounting of your motors.
 */

// Femur → Vertical lifting joint (middle joint)
int femurDir[16] = {
  0, +1, 0, 0, 0, +1, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0
};

// Tibia → Foot joint (lower joint)
int tibiaDir[16] = {
  -1, 0, 0, 0, -1, 0, 0, 0, 0, 0, +1, 0, 0, 0, +1, 0
};

// Coxa → Arm joint (horizontal rotation)
int coxaDir[16] = {
  0, 0, +1, 0, 0, 0, +1, 0, +1, 0, 0, 0, +1, 0, 0, 0
};


/* ================= MOTION PARAMETERS =================
 * liftAmount → Vertical clearance of the leg during lift
 * stepAmount → Forward/Backward swing distance
 * stepDelay  → Delay between micro-steps (milliseconds)
 *
 * These values define speed, smoothness, and stability.
 * Tune carefully to avoid mechanical stress.
 */
int liftAmount = 200;
int stepAmount = 150;
int stepDelay  = 250;
int magDelay = 250;


/********************************************************************
 * Configuration Validator
 * --------------------------------------------------
 * Checks whether all PWM values are inside safe limits.
 * If any invalid value is detected, firmware must halt.
 *
 * Return:
 *   true  → Configuration is valid
 *   false → Invalid calibration detected
 ********************************************************************/
bool validateConfiguration() {

  // Validate stand positions
  for (int i = 0; i < 16; i++) {
    if (standPos[i] != 0) {
      if (standPos[i] < SERVO_MIN || standPos[i] > SERVO_MAX) {
        return false;
      }
    }
  }

  return true;  // Configuration is safe
}
