#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

// ================= SERVO SAFETY LIMITS =================
#define SERVO_MIN 200
#define SERVO_MAX 550

// ================= LEG STRUCT =================
struct Leg {
  int coxa;
  int femur;
  int tibia;
};

// ================= CONFIGURATION DECLARATIONS =================
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
bool validateConfiguration();

#endif