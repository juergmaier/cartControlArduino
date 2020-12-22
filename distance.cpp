// 
// 
// 
#include "distance.h"
#include "cartControlArduino.h"
#include "Drive.h"
#include "Table.h"

#include <EEPROM.h>
#include <Servo.h>

#define min(a,b) ((a)<(b)?(a):(b))
#define cos30 0.866					// used to calc obstacle height
Servo swipeServos[SWIPE_SERVOS_COUNT];

// Configurable Values
int swipeServoStartAngle = 30;	// swipe servo start degrees
int swipeServoRange = 110;		// swipe servo range

//
// const for calculating distance from infrared sensor raw values
// 
ir_sensor GP2Y0A21YK = { 5461.0, -17.0, 2.0 };
ir_sensor GP2Y0A41YK = { 1500.0, -11.0, 0 };

// infrared distance sensors
irSensorStepValuesType irSensorStepData[IR_SENSORS_COUNT][NUM_MEASURE_STEPS];
int irSensorStepRaw[IR_SENSORS_COUNT][NUM_REPETITIONS_IR_MEASURE];	// raw values current step

byte irSensorObstacleMaxValue;
byte irSensorObstacleMaxSensor;
byte irSensorAbyssMaxValue;
byte irSensorAbyssMaxSensor;

// the list of used servos and sensors in current cartDirection
boolean servoInvolved[SWIPE_SERVOS_COUNT];
int involvedIrSensors[MAX_INVOLVED_IR_SENSORS];
int numInvolvedIrSensors;
int swipeStep;

// the swiping servo variables
int swipeDirection;			// to swipe servo horn from left to right (+1) and back (-1)
int currentMeasureStep;	    // 
int nextMeasureStep;
int nextSwipeServoAngle;
long swipeStepStartMillis = 0;

// the ultrasonic distance sensors
ultrasonicDistanceSensorDefinition ultrasonicDistanceSensor[ULTRASONIC_DISTANCE_SENSORS_COUNT]{
	// sensorId, enabled, name,                 trigPin, echoPin, distanceCorrection
	{  0,        true,    "us front left left",    2,       3,       0 },
	{  1,        true,    "us front left center",  4,       5,       0 },
	{  2,        true,    "us front right center", 6,       7,       0 },
	{  3,        true,    "us front right right",  8,       9,       0 },
};

int ultrasonicDistanceSensorValues[ULTRASONIC_DISTANCE_SENSORS_COUNT];
int ultrasonicDistanceSensorValidity;


// Achtung: Adafruit DC Motors benutzen pin 4,7,8 und 12
// Bei Kollision in der Benutzung der Pins st?rzt der Arduino ab
// Mega-PWM-Pins = 2..13, 44,45,46
static servoDefinition servoDefinitions[SWIPE_SERVOS_COUNT]{
	//    Id, name, installed, pin, degree offset (clockwise +)
		{ FL, "FL", true, 11,   0 },
		{ FC, "FC", true, 12,   0 },
		{ FR, "FR", true, 13,  10 },
		{ BL, "BL", true,  6, -8 },
		{ BC, "BC", true,  9, -12 },
		{ BR, "BR", true, 10,  30 }
};


// sensorId, sensorName, sensorTyp, sensorRange, installed, pin, servoId, floorDistance
// swiped sensors should show 15 cm, fixed sensors 18 cm
irSensorDefinition irSensorDefinitions[IR_SENSORS_COUNT]{
//	  sensorId;          sensorName[20];         TYPE; swipe; installed; sensorPin; servoId
	{ FRONT_LEFT,        "swipeFrontLeft",     A41, true,  true,      A2,         FL},
	{ FRONT_CENTER,      "swipeFrontCenter ",  A41, true,  true,      A7,         FC},
	{ FRONT_RIGHT,       "swipeFrontRight",    A41, true,  true,      A3,         FR},
	{ BACK_LEFT,         "swipeBackLeft",      A21, true,  true,      A8,         BL},
	{ BACK_CENTER,       "swipeBackCenter",    A21, true,  true,      A12,        BC},
	{ BACK_RIGHT,        "swipeBackRight",     A21, true,  true,      A10,        BR},
	{ LEFT_SIDE_FRONT,   "staticLeftFront",    A21, false, true,      A0,         -1},
	{ LEFT_SIDE_BACK,    "staticLeftBack",     A21, false, true,      A9,         -1},
	{ RIGHT_SIDE_FRONT,  "staticRightFront",   A21, false, true,      A1,         -1},
	{ RIGHT_SIDE_BACK,   "staticRightBack ",   A21, false, true,      A11,        -1}
};

byte irSensorReferenceDistances[IR_SENSORS_COUNT][NUM_MEASURE_STEPS];

// DEFINITION of global variables
// sensorId with measured closest obstacle
//int maxObstacleSensorId;

// sensorId with measured largest abyss
//int maxAbyssSensorId;



// distance 021 20..80 cm
// convert infrared sensor values to mm
// mark invalid values as DISTANCE_UNKNOWN
int analogToDistanceA21(ir_sensor sensor, int adc_value)
{
	if (adc_value < 50 || adc_value > 800)
	{
		if (verbose) {
			Serial.print(F("A21, raw value ")); Serial.print(adc_value); Serial.print(F(" out of scope (50..800): "));
			Serial.print(adc_value);
			Serial.println();
		}
		return DISTANCE_UNKNOWN;	//not a valid raw value
	}
	return (sensor.a / (adc_value + sensor.b) - sensor.k) * 10;	// dist in mm
}


// convert infrared sensor A41 values to mm
// mark invalid values as DISTANCE_UNKNOWN
int analogToDistanceA41(int adc_value)
{
	if (adc_value < 50 || adc_value > 800)
	{
		if (verbose) {
			Serial.print(F("A41, raw value ")); Serial.print(adc_value); Serial.print(F(" out of scope (50..800): "));
			Serial.print(adc_value);
			Serial.println();
		}
		return DISTANCE_UNKNOWN;	//not a valid raw value
	}
	
	return 13 / (adc_value * 0.00048828125);
}


////////////////////////////////////////////////////////////////////////
void setupSwipeServos(int dockingSwitch) {

	Serial.println(F("setup swipe servos"));
	int Pin;
	for (int sensorId = 0; sensorId < SWIPE_SERVOS_COUNT; sensorId++) {
	//for (int sensorId = 2; sensorId < 3; sensorId++) {
		Pin = servoDefinitions[sensorId].servoPin;
		if (servoDefinitions[sensorId].installed) pinMode(Pin, OUTPUT);
	}

	// check for startup with activated docking switch
	// in that case do not run the initial check on the swipe servos as we are right 
	// in front of the power station
	if (dockingSwitch != 99) {

		Serial.print(F("for ir sensor testing ignore dockingSwitch value: ")); Serial.println(dockingSwitch);
		Serial.println(F("now do some repeated ir distance measures to verify infrared sensors"));
		Serial.println(F("move swipe servo to middle position for checking correct horn position"));
		for (int sensorId = 0; sensorId < IR_SENSORS_COUNT; sensorId++) {
			
			Serial.print(F("sensor: ")); Serial.println(irSensorDefinitions[sensorId].sensorName);

			if (irSensorDefinitions[sensorId].swipe) {
				swipeServos[sensorId].attach(servoDefinitions[sensorId].servoPin);
				swipeServos[sensorId].write(90+servoDefinitions[sensorId].servoDegreeOffset);
				delay(70);   // let the swipe servo reach its destination
			}

			// let the sensor adjust
			for (int i = 0; i < IR_MIN_READS_TO_ADJUST; i++){
				analogRead(irSensorDefinitions[sensorId].sensorPin);
				delay(2);
			}

			int distRaw;
			int distMm;
			int distMin = 500;
			int distMax = 0;

			for (int i = 0; i < 10; i++) {
				distRaw = analogRead(irSensorDefinitions[sensorId].sensorPin);
				if (irSensorDefinitions[sensorId].sensorType == A21) {
					distMm = analogToDistanceA21(GP2Y0A21YK, distRaw);
				} else {
					distMm = analogToDistanceA41(distRaw);
				}
				if (distMm < distMin) {distMin = distMm;}
				if (distMm > distMax) {distMax = distMm;}
				
				Serial.print(F("distRaw: ")); Serial.print(distRaw);
				Serial.print(F(", distMm: ")); Serial.print(distMm);
				Serial.println();
				delay(DELAY_BETWEEN_ANALOG_READS);
			}
			Serial.print(F("distMin: ")); Serial.print(distMin); 
			Serial.print(F(", distMax: ")); Serial.print(distMax);
			Serial.print(F(", range: ")); Serial.print(distMax-distMin);
			Serial.println();
		}
	}

	stopSwipe();

	Serial.println(F("setup swipe servos done"));
}



void attachServo(int servoId) {
	if (!swipeServos[servoId].attached()) {
		swipeServos[servoId].attach(servoDefinitions[servoId].servoPin);
	}
}

/////////////////////////
/////////////////////////
void nextSwipeServoStep() {

	int relAngle;

	nextMeasureStep = currentMeasureStep + swipeDirection;
	
	Serial.print(F("nextSwipeStep, current: ")); Serial.print(currentMeasureStep);
	Serial.print(F(", next: ")); Serial.print(nextMeasureStep);
	Serial.println();

	if (nextMeasureStep == NUM_MEASURE_STEPS - 1) {
		swipeDirection = -1;
	}
	if (nextMeasureStep == 0) {
		swipeDirection = 1;
	}

	if (currentMeasureStep == 0 || currentMeasureStep == NUM_MEASURE_STEPS) {
		// all scan steps done, update cartControl
		//Serial.print(F("!F0,from_drive, Step")); Serial.println(currentMeasureStep);
	}

	// the relative Angle for the measuring
	// each servo can have an individual degree offset compensating for servo horn mount
	// and servo build differences.
	// the offset can be used to get an identical direction of the sensors for each measure step
	// correct offset can easily be verified looking at the sensor in the "rest" position (swipeStartAngle)
	// adjust the offset in the servoDefinitions
	relAngle = int(nextMeasureStep * (swipeServoRange / (NUM_MEASURE_STEPS-1)));

	// for all involved servos of the requested cartDirection rotate to next step
	for (int servoId = 0; servoId < SWIPE_SERVOS_COUNT; servoId++) {
		if (servoInvolved[servoId]) {

			// swipe servo for measurement in different directions
			attachServo(servoId);		// conditional attach

			nextSwipeServoAngle =  relAngle 
				+ swipeServoStartAngle
				+ servoDefinitions[servoId].servoDegreeOffset;

			swipeServos[servoId].write(nextSwipeServoAngle);
		}
	}
	//Serial.print(F("swipe servo moved to nextMeasureStep: ")); Serial.print(nextMeasureStep);
	//Serial.println();

	swipeStepStartMillis = millis();
	return;
}



///////////////////////////////////////////////////
// check timestamps in distance sensor values 
///////////////////////////////////////////////////
bool isIrSensorDataCurrent() {

	unsigned long oldestMeasureMillis = pow(2, 31);
	int oldestMeasureSensor;
	int oldestMeasureStep;
	bool isCurrent;

	// check involved irSensors
	for (int item = 0; item < numInvolvedIrSensors; item++) {

		int sensorId = involvedIrSensors[item];
		int numStepsToCheck = irSensorDefinitions[sensorId].swipe ? NUM_MEASURE_STEPS : 1;

		for (int step = 0; step < numStepsToCheck; step++) {
			if (irSensorStepData[sensorId][step].lastMeasureMillis < oldestMeasureMillis) {
				oldestMeasureMillis = irSensorStepData[sensorId][step].lastMeasureMillis;
				oldestMeasureSensor = sensorId;
				oldestMeasureStep = step;
			};
		}
		isCurrent = (millis() - oldestMeasureMillis) < 2500;

		if (false) {
			Serial.print(millis()-moveRequestReceivedMillis); Serial.print(", ");
			if (isCurrent) {
				Serial.print(F("swipe sensor current, oldest: ")); Serial.print(millis() - oldestMeasureMillis);	
			} else {
				Serial.print(F("swipe sensor data out of date, oldest: ")); Serial.print(millis() - oldestMeasureMillis);
			}
			Serial.print(F(", limit: 2500"));
			Serial.print(F(", sensor: ")); Serial.print(getIrSensorName(oldestMeasureSensor));
			Serial.print(F(", step: ")); Serial.print(oldestMeasureStep);
			Serial.println();
		}
	}
	return isCurrent;
}



//////////////////////////////////////////////
// sort an array, modifies the passed in array
//////////////////////////////////////////////
void bubbleSort(int arr[], int numItems) {
	boolean swapped = true;
	int j = 0;
	int tmp;

	// repeat until no swappes occur anymore
	while (swapped) {
		swapped = false;
		j++;
		for (int i = 0; i < numItems - j; i++) {
			if (arr[i] > arr[i + 1]) {
				tmp = arr[i];
				arr[i] = arr[i + 1];
				arr[i + 1] = tmp;
				swapped = true;
			}
		}
	}
	//Serial.print(F("sorted: "));
	//for (int sensorId = 0; sensorId < numItems; sensorId++) {
	//	Serial.print(arr[sensorId]); Serial.print(", ");
	//}
	//Serial.println();
}


// swiping servos should have arrived at current step
// for all involved sensors do some reads to let sensor adjust to new distance
// then do a number of reads to allow to calc a median (done later in processing of the raw data)
// check range of repeated reads and retry max 3 times if values have a high deviation
// the raw values are stored in irSensorData array and the time of last measure is saved
void readIrSensorValues(int swipeStep) {

	int distRaw;
	int sensorId;
	
	for (int item = 0; item < numInvolvedIrSensors; item++) {
		sensorId = involvedIrSensors[item];
		// for static sensors set swipe step to 0
		if ( ! irSensorDefinitions[sensorId].swipe) swipeStep = 0;

		// so some reads to let the sensor adjust to the new position
		for (int i = 0; i < IR_MIN_READS_TO_ADJUST; i++){
			analogRead(irSensorDefinitions[sensorId].sensorPin);
			delayMicroseconds(600);
		}

		int minValue = 1028;
		int maxValue = 0;
		int numTries = 0;
		int rangeLimit = 50;

		// repeat reading the sensor if we see a high deviation in values
		while (numTries < 3) {
			// now do a number of reads for getting a median and a value range
			for (int m = 0; m < NUM_REPEATED_MEASURES; m++) {

				distRaw = analogRead(irSensorDefinitions[sensorId].sensorPin);
				irSensorStepRaw[sensorId][m] = distRaw;
				if (distRaw < minValue) minValue = distRaw;
				if (distRaw > maxValue) maxValue = distRaw;

				/*
				// log every measurement in sensor test and verbose mode
				if (sensorInTest == sensorId && verbose) {
					Serial.print(getInfraredSensorName(sensorId));
					Serial.print(F(", swipeDir: "); Serial.print(swipeDirection);
					Serial.print(F(", swipeStep:"); Serial.print(swipeStep);
					Serial.print(F(", analogIn: A"); Serial.print(irSensorDefinitions[sensorId].sensorPin - 54);
					Serial.print(F(", measure: "); Serial.print(m);
					Serial.print(F(", raw: "); Serial.print(distRaw);
					Serial.println();
				}
				*/	
				delayMicroseconds(500);
			}
			if (abs(maxValue - minValue) < rangeLimit) break;
			minValue = 1028;
			maxValue = 0;
			numTries++;
		}
		if (numTries > 2) {
			prt("could not read consistent values: "); pr(getIrSensorName(sensorId));
			for (int i = 0; i < NUM_REPEATED_MEASURES; i++) {
				pr(irSensorStepRaw[sensorId][i]); prt(", ");
			}
			prl();
		}

		// set time of last read
		irSensorStepData[sensorId][swipeStep].lastMeasureMillis = millis();		
	}
}


/////////////////////////////////////////////////////////
// based on the collected raw values of the sensors in irSensorData
// find the median raw value
// calculate the distance in mm
// eval the new min/max values of the sensor
/////////////////////////////////////////////////////////
void processNewRawValues(int swipeStep) {

	byte sensorId;
	int medianRaw;
	byte distMm;
	float heightFactor;

	// process distance data only for sensors involved in movement
	for (byte item = 0; item < numInvolvedIrSensors; item++) {
		sensorId = involvedIrSensors[item];

		byte refDist = irSensorReferenceDistances[sensorId][swipeStep];
		byte obstacleHeight = 0;
		byte abyssDepth = 0;

		// for static sensors set swipe step to 0
		if (!irSensorDefinitions[sensorId].swipe) swipeStep = 0;

		// for each sensor sort the raw reads and use median as distance
		bubbleSort(irSensorStepRaw[sensorId], NUM_REPEATED_MEASURES);
		medianRaw = irSensorStepRaw[sensorId][int(NUM_REPEATED_MEASURES / 2)];

		if (false) {
			prt("raw values: ");
			for (int i = 0; i < NUM_REPEATED_MEASURES; i++) {
				pr(irSensorStepRaw[sensorId][i]); prt(", ");
			}
			prt("range: ");
			pr(irSensorStepRaw[sensorId][NUM_REPEATED_MEASURES-1]-irSensorStepRaw[sensorId][0]);
			prt(", median: "); pr(medianRaw);
			prl();
		}

		// calculate mm from raw median for the sensortype used
		if (irSensorDefinitions[sensorId].sensorType == A41) {
			distMm = analogToDistanceA41(medianRaw);
		}
		if (irSensorDefinitions[sensorId].sensorType == A21) {
			distMm = analogToDistanceA21(GP2Y0A21YK, medianRaw);
		}

		// check for valid distance and calculate obstacleHeight / abyssDepth
		if (distMm == DISTANCE_UNKNOWN) {
			// set distance to reference value, results in 0 obstacleHeight / 0 abyssDepth
			distMm = refDist; 
		}
		// set factor to calculate height/depth from distance
		heightFactor = irSensorDefinitions[sensorId].swipe ? cos30: 1;

		// set obstacle or abyss value
		if (distMm < refDist) obstacleHeight = (refDist - distMm) * heightFactor;	// positive value
		if (distMm > refDist) abyssDepth = (distMm - refDist) * heightFactor;		// positive value

		// update data
		//irSensorStepValuesType irSensorSwipeData[IR_SENSORS_COUNT][NUM_MEASURE_STEPS];
		irSensorStepData[sensorId][swipeStep].distMm = distMm;
		irSensorStepData[sensorId][swipeStep].obstacleHeight = obstacleHeight;
		irSensorStepData[sensorId][swipeStep].abyssDepth = abyssDepth;

		// in sensor test and verbose mode print the summary
		if (moveType == SENSORTEST && verbose) {		// in sensor test and verbose mode 
			pr(getIrSensorName(sensorId));
			prt(", swipeStep: "); pr(swipeStep);
			prt(", distance: "); pr(distMm);
			prt(", reference: "); pr(irSensorReferenceDistances[sensorId][swipeStep]);
			prt(", diff: "); pr(irSensorReferenceDistances[sensorId][swipeStep] - distMm);
			prl();

			if (false) {
				prt("dist: ");
				for (int step = 0; step < NUM_MEASURE_STEPS; step++) {
					pr(irSensorReferenceDistances[sensorId][step] - irSensorStepData[sensorId][step].distMm); prt(", ");
				}
				prl();
			}
		}
	}

	// log the floor offset of involved sensors
	if (plannedCartMovement != STOP) {

		for (int item = 0; item < numInvolvedIrSensors; item++) {
			sensorId = involvedIrSensors[item];

			// check for last or first position in swipe
			if ((currentMeasureStep == NUM_MEASURE_STEPS) 
				|| currentMeasureStep == 0
				|| !irSensorDefinitions[sensorId].swipe) {

				// do not log the first summary after the command start
				if (millis() - moveRequestReceivedMillis > 500) {

					// in case of a sensor test log the measured distances too
					if (moveType == SENSORTEST && sensorId == sensorInTest) {
						logIrDistanceValues(sensorId);
					}

					prt("swipe summary: "); pr(getIrSensorName(sensorId));
					prt(", sensor obstacle max: "); pr(irSensorObstacleMaxValue);
					prt(", sensor abyss max: "); pr(irSensorAbyssMaxValue);
					prl();
				}
			}
		}
	}
}


void logMeasureStepResults() {

	int sensorId;
	int step;

	// !F1,[<sensorId>,<step>,<obstacleHeight>,<abyssDepth>]*involvedSensors
	pr("!F1");
	for (int item = 0; item < numInvolvedIrSensors; item++) {
		sensorId = involvedIrSensors[item];
		step = irSensorDefinitions[sensorId].swipe ? currentMeasureStep : 0;
		pr(","); pr(sensorId);
		pr(","); pr(step);
		pr(","); pr(irSensorStepData[sensorId][step].obstacleHeight);
		pr(","); pr(irSensorStepData[sensorId][step].abyssDepth);
	}
	prl();
}


// set max obstacle/abyss with new measured values
void setIrSensorsMaxValues() {

	// update max values of obstacle and abyss with new data
	irSensorObstacleMaxValue = 0;
	irSensorAbyssMaxValue = 0;
	int sensorId;
	for (int item = 0; item < numInvolvedIrSensors; item++) {
		sensorId = involvedIrSensors[item];

		// check for swiping or static sensor
		int numSteps = irSensorDefinitions[sensorId].swipe? NUM_MEASURE_STEPS : 1;

		for (int step = 0; step < numSteps; step++) {
			if (irSensorStepData[sensorId][swipeStep].obstacleHeight > irSensorObstacleMaxValue) {
				irSensorObstacleMaxValue = irSensorStepData[sensorId][swipeStep].obstacleHeight;
				irSensorObstacleMaxSensor = sensorId;
			}
			if (irSensorStepData[sensorId][swipeStep].abyssDepth > irSensorAbyssMaxValue) {
				irSensorAbyssMaxValue = irSensorStepData[sensorId][swipeStep].abyssDepth;
				irSensorAbyssMaxSensor = sensorId;
			}
		}
	}
	Serial.print(F("max obstacle: ")); Serial.print(irSensorObstacleMaxValue);
	Serial.print(F(", sensor: ")); Serial.print(getIrSensorName(irSensorObstacleMaxSensor));
	Serial.print(F(", max abyss: ")); Serial.print(irSensorAbyssMaxValue);
	Serial.print(F(", sensor: ")); Serial.print(getIrSensorName(irSensorAbyssMaxSensor));
	Serial.println();
}



/////////////////////////////
void logIrDistanceValues(int sensorId) {

	byte distance;

	// check type of sensor, swiping or fixed
	byte numMeasureSteps = irSensorDefinitions[sensorId].swipe ? NUM_MEASURE_STEPS : 1;

	// !F3,<sensorId>,<NUM_MEASURE_STEPS>,[<distanceValues>,]
	Serial.print("!F3,"); Serial.print(sensorId);
	Serial.print(","); Serial.print(numMeasureSteps);

	// add the measured distances
	for (int swipeStep = 0; swipeStep < numMeasureSteps; swipeStep++) {
		Serial.print(","); Serial.print(irSensorStepData[sensorId][swipeStep].distMm);
	}
	Serial.println();
}



void resetServo(int servoId) {

	// move servo to start position (min)
	int offset = servoDefinitions[servoId].servoDegreeOffset;
	swipeServos[servoId].write(swipeServoStartAngle + offset);

	// will cause first move to servo reset position where it is after a stop
	currentMeasureStep = 0;
	nextMeasureStep = 0;
	swipeDirection = 1;
}


//////////////////////////////////////////
// stop the distance measure swipe
//////////////////////////////////////////
void stopSwipe() {

	for (int servoId = 0; servoId < SWIPE_SERVOS_COUNT; servoId++) {

		resetServo(servoId);
	}

	// allow servos to move to the reset position before detaching them
	delay(100);

	for (int servoId = 0; servoId < SWIPE_SERVOS_COUNT; servoId++) {
		swipeServos[servoId].detach();
	}
	//Serial.println("distance swipe stopped");
}


const char* getIrSensorName(int sensorId) {
	return irSensorDefinitions[sensorId].sensorName;
}


const char* getUsSensorName(int sensorId) {
	return ultrasonicDistanceSensor[sensorId].sensorName;
}


void loadFloorReferenceDistances() {

	int eepromStartAddr;
	int byteValue;
	int adjustedValue;

	// load saved reference distances from EEPROM
	Serial.println(F("ir sensor reference distances from eeprom "));
	for (int sensorId = 0; sensorId < IR_SENSORS_COUNT; sensorId++) {
	
		//Serial.print(sensorId); Serial.print(": "); Serial.print(irSensorDefinitions[sensorId].sensorName);
		//Serial.print(": ");
		int eepromStartAddr = sensorId * NUM_MEASURE_STEPS;

		pr("!F2,"); pr(sensorId); pr(","); pr(NUM_MEASURE_STEPS); pr(",");

		for (int swipeStep = 0; swipeStep < NUM_MEASURE_STEPS; swipeStep++) {
			byteValue = EEPROM.read(eepromStartAddr + swipeStep);
			adjustedValue = byteValue < 0 ? byteValue + 128 : byteValue;
			irSensorReferenceDistances[sensorId][swipeStep] = adjustedValue;
			//Serial.print(adjustedValue); Serial.print(", ");
			pr(adjustedValue); pr(",");
		}
		prl();

	}
}
