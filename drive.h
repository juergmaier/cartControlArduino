// Fahren.h

#ifndef _FAHREN_h
#define _FAHREN_h

#include "arduino.h"
#include "distance.h"

extern char receivedChars[64];
extern int numChars;
extern int targetYaw;
//extern boolean inRotation;


//given by cartControl with message 0
extern int FLOOR_MAX_OBSTACLE;
extern int FLOOR_MAX_ABYSS;

extern unsigned long _last12VCheckMillis;
extern int _mVolts;
extern volatile unsigned long wheelPulseCounter;
extern unsigned long moveRequestReceivedMillis;

extern int MIN_MEASURE_CYCLE_DURATION;

extern boolean servoInvolved[];
extern int involvedIrSensors[];
extern int numInvolvedIrSensors;

enum MOTORS { MOTOR_FRONT_RIGHT, MOTOR_FRONT_LEFT, MOTOR_BACK_RIGHT, MOTOR_BACK_LEFT, MOTORS_COUNT };
enum SPEED_PHASE {ACCELERATE, CRUISE, DECELERATE};
extern int _speedUnifyer[MOTORS_COUNT];


// initialise move
extern void setupFahren();

// driving direction
extern void setPlannedCartMove(MOVEMENT, int, int, int, bool);
extern void setPlannedCartRotation(MOVEMENT, int, int, int);
extern MOVEMENT plannedCartMovement;
extern MOVEMENT activeCartMovement;

// stop base
extern void stopCart(bool, String);

// drive
extern void handleCartMovement();
//extern void setInvolvedIrSensors(MOVEMENT plannedCartDirection, bool moveProtected);

extern int checkUltrasonicDistances();

#endif

