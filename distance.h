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


#define DISTANCE_UNKNOWN 999

#define MAX_INVOLVED_SENSORS 6
#define MAX_REPEATED_MEASURES 10
#define MAX_RAW_RANGE 50

// do not change the order or make sure to adjust code with all occurrences
typedef enum FLOOR_DISTANCE_SENSORS {
	FRONT_LEFT, FRONT_CENTER, FRONT_RIGHT,
	BACK_LEFT, BACK_CENTER, BACK_RIGHT,
	LEFT_SIDE_FRONT, LEFT_SIDE_BACK, 
	RIGHT_SIDE_FRONT, RIGHT_SIDE_BACK,
	FLOOR_SENSORS_COUNT
} FLOOR_DISTANCE_SENSOR;


enum FLOOR_SENSOR_TYPES { A41, A21, SENSOR_TYPEN_COUNT };		// Infrared sensor type
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


typedef struct floorSensorDefinition {
	int sensorId;
	char sensorName[20];
	FLOOR_SENSOR_TYPES sensorType;
	bool swipe;
	bool installed;
	int sensorPin;
	int servoID;
} floorSensorDefinition;

typedef struct floorSensorSwipeStepDataType {
	int raw[MAX_REPEATED_MEASURES];
	int range;
} floorSensorSwipeStepDataType;

typedef struct floorSensorValues {
	unsigned long lastMeasurementTime;
	int distance;
} floorSensorValues;


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
extern int floorObstacleMax;
extern int floorObstacleMaxId;
extern int floorAbyssMax;
extern int floorAbyssMaxId;

// variables for max distances
extern int maxDistanceSensorId;
extern int maxDistance;


//extern int swipeDirection;		// to swipe from right to left and back from left to right
extern int currentMeasureStep;	    // for the swiping distance sensors the current swipe step
extern int nextMeasureStep;
extern int swipeDirection;
extern int nextSwipeServoAngle;

extern int MIN_MEASURE_CYCLE_DURATION;

extern int loopCount;

extern floorSensorDefinition floorSensorDefinitions[];
extern int floorSensorReferenceDistances[FLOOR_SENSORS_COUNT][NUM_MEASURE_STEPS];

extern floorSensorSwipeStepDataType sensorDataSwipeStep[FLOOR_SENSORS_COUNT];

extern int ultrasonicDistanceSensorValues[ULTRASONIC_DISTANCE_SENSORS_COUNT];
extern int ultrasonicDistanceSensorValidity;


// DECLARATION of global functions in distance.cpp
extern bool checkFloorSensors(MOVEMENT);
extern const char* getInfraredSensorName(int);
extern const char* getUltrasonicSensorName(int);

extern void setupSwipeServos(int);
extern int readDistanceSensorRawValues(MOVEMENT activeCartMovement);
extern void evalObstacleOrAbyss(MOVEMENT activeCartMovement);
extern void logFloorOffset(int sensorId);
extern void logDistanceValues(int sensorId);
extern void stopSwipe();
extern void nextSwipeServoStep();
extern void saveFloorReferenceToEEPROM(int sensorId);

#endif

