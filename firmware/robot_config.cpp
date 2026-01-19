/************************************************************
 * File     : robot_config.cpp
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
 * This file contains all hardware calibration parameters and
 * mechanical configuration data for the quadruped robot with
 * electromagnetic feet support.
 *
 * WARNING:
 * Incorrect values in this file may damage servos, electromagnets,
 * or mechanical structures. Always keep all active PWM values
 * within:
 *
 *        SERVO_MIN (200)  ≤  PWM  ≤  SERVO_MAX (550)
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
  350, 350, 300, 0,
  450, 350, 400, 0,
  375, 430, 250, 0,
  550, 350, 350, 0
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
Leg FL = {4, 5, 6};     // Front Left Leg
Leg FR = {2, 1, 0};     // Front Right Leg
Leg BL = {12, 13, 14};  // Back Left Leg
Leg BR = {10, 9, 8};    // Back Right Leg


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
  0, -1, 0, 0, 0, +1, 0, 0, 0, +1, 0, 0, 0, -1, 0, 0
};

// Tibia → Foot joint (lower joint)
int tibiaDir[16] = {
  +1, 0, 0, 0, 0, 0, -1, 0, -1, 0, 0, 0, 0, 0, +1, 0
};

// Coxa → Arm joint (horizontal rotation)
int coxaDir[16] = {
  0, 0, +1, 0, +1, 0, 0, 0, 0, 0, +1, 0, +1, 0, 0, 0
};


/* ================= MOTION PARAMETERS =================
 * liftAmount → Vertical clearance of the leg during lift
 * stepAmount → Forward/Backward swing distance
 * stepDelay  → Delay between micro-steps (milliseconds)
 *
 * These values define speed, smoothness, and stability.
 * Tune carefully to avoid mechanical stress.
 */
int liftAmount = 150;
int stepAmount = 150;
int stepDelay  = 1000;
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
