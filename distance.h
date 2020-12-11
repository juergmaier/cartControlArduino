// Distanz.h

#ifndef _DISTANZ_h
#define _DISTANZ_h

#include <Wire.h>
#include "arduino.h"

#include "cartControlArduino.h"
#include "drive.h"



//
// structure and parameter for calculating infrared distance sensors raw values into cm
//
typedef struct ir_sensor {
	float a;
	float b;
	float k;
} ir_sensor;


#define DISTANCE_UNKNOWN 255

#define MAX_INVOLVED_IR_SENSORS 6
#define NUM_REPETITIONS_IR_MEASURE 10
#define MAX_RAW_RANGE 60

// do not change the order or make sure to adjust code with all occurrences
typedef enum FLOOR_DISTANCE_SENSORS {
	FRONT_LEFT, FRONT_CENTER, FRONT_RIGHT,
	BACK_LEFT, BACK_CENTER, BACK_RIGHT,
	LEFT_SIDE_FRONT, LEFT_SIDE_BACK, 
	RIGHT_SIDE_FRONT, RIGHT_SIDE_BACK,
	IR_SENSORS_COUNT
} FLOOR_DISTANCE_SENSOR;


enum IR_SENSOR_TYPES { A41, A21, SENSOR_TYPEN_COUNT };		// Infrared sensor type
enum FLOOR_SENSOR_INFO { SENSOR_ID, SENSOR_TYPE, SENSOR_PIN, SWIPE_SERVO_ID, SENSOR_INFO_COUNT };


// the swipe servos
typedef enum SWIPE_SERVOS {
	FL, FC, FR, BL, BC, BR, 
	SWIPE_SERVOS_COUNT
} SWIPE_SERVO;

enum DISTANCE_LIMIT_TYPE { MINIMUM, MAXIMUM };

enum DISTANCE_SENSOR_TYPE {INFRARED, ULTRASONIC};

typedef struct servoDefinition {
	int servoID;
	char servoName[3];
	boolean installed;
	int servoPin;
	int servoDegreeOffset;
} servoDefinition;


typedef struct irSensorDefinition {
	int sensorId;
	char sensorName[20];
	IR_SENSOR_TYPES sensorType;
	bool swipe;
	bool installed;
	int sensorPin;
	int servoID;
} irSensorDefinition;

//typedef struct irSensorSwipeStepDataType {
//	byte raw[NUM_REPETITIONS_IR_MEASURE];
//} irSensorSwipeStepDataType;

typedef struct irSensorStepValuesType {
	byte distMm;
	byte obstacleHeight;
	byte abyssDepth;
	unsigned long lastMeasureMillis;
} irSensorStepValuesType;


// Ultrasonic Distance Sensor
typedef struct {
	int sensorId;
	bool installed;
	char sensorName[25];
	int trigPin;			// currently only used in Arduino Nano for evaluating the distance
	int echoPin;			// currently only used in Arduino Nano for evaluating the distance
	int distanceCorrection;
}ultrasonicDistanceSensorDefinition;

// the ultrasonic sensors
typedef enum {
	FLL, FLC, FRC, FRR,
	ULTRASONIC_DISTANCE_SENSORS_COUNT
} ULTRASONIC_DISTANCE_SENSORS;


// DECLARATION of global variables of distance.cpp
extern byte irSensorObstacleMaxValue;
extern byte irSensorObstacleMaxSensor;
extern byte irSensorAbyssMaxValue;
extern byte irSensorAbyssMaxSensor;

// variables for max distances
extern int maxDistanceSensorId;
extern int maxDistance;

//extern int swipeDirection;		// to swipe from right to left and back from left to right
extern int currentMeasureStep;	    // for the swiping distance sensors the current swipe step
extern int nextMeasureStep;
extern int swipeDirection;
extern int nextSwipeServoAngle;
extern long swipeStepStartMillis;

extern int MIN_MEASURE_CYCLE_DURATION;
extern int IR_MIN_READS_TO_ADJUST;		// repeat a number of reads before using the ir sensor values
extern int IR_MIN_WAIT_SWIPE_SERVO;		// minimal wait time after swipe servo move before reading values
extern int loopCount;

extern irSensorDefinition irSensorDefinitions[];
extern byte irSensorReferenceDistances[IR_SENSORS_COUNT][NUM_MEASURE_STEPS];

extern byte irSensorStepRaw[IR_SENSORS_COUNT][NUM_REPETITIONS_IR_MEASURE];

extern int ultrasonicDistanceSensorValues[ULTRASONIC_DISTANCE_SENSORS_COUNT];
extern int ultrasonicDistanceSensorValidity;


// DECLARATION of global functions in distance.cpp
extern bool checkFloorSensors(MOVEMENT);
extern const char* getIrSensorName(int);
extern const char* getUsSensorName(int);

extern void setupSwipeServos(int);
extern void readIrSensorValues(int swipeStep);
extern void processNewRawValues(int swipeStep);
extern void logMeasureStepResults();
extern void setIrSensorsMaxValues();
extern bool isIrSensorDataCurrent();

extern int maxRangeIrSensorRawValues(MOVEMENT activeCartMovement);
extern void logIrDistanceValues(int sensorId);
extern void stopSwipe();
extern void nextSwipeServoStep();
extern void saveFloorReferenceToEEPROM(int sensorId);


#endif

