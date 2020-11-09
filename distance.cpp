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

Servo swipeServos[SWIPE_SERVOS_COUNT];

// Configurable Values
int swipeServoStartAngle = 30;	// swipe servo start degrees
int swipeServoRange = 130;		// swipe servo range

//
// const for calculating distance from infrared sensor raw values
// 
ir_sensor GP2Y0A21YK = { 5461.0, -17.0, 2.0 };
ir_sensor GP2Y0A41YK = { 1500.0, -11.0, 0 };

// infrared distance sensors
floorSensorValues sensorData[FLOOR_SENSORS_COUNT][NUM_MEASURE_STEPS];
floorSensorSwipeStepDataType sensorDataSwipeStep[FLOOR_SENSORS_COUNT];
int floorObstacleMax;
int floorObstacleMaxId;
int floorAbyssMax;
int floorAbyssMaxId;

// the list of used servos and sensors in current cartDirection
boolean servoInvolved[SWIPE_SERVOS_COUNT];
boolean sensorInvolved[FLOOR_SENSORS_COUNT];
int swipeStep;

// the swiping servo variables
int swipeDirection;			// to swipe servo horn from left to right (+1) and back (-1)
int currentMeasureStep;	    // 
int nextMeasureStep;
int nextSwipeServoAngle;

int sensorObstacleMax[FLOOR_SENSORS_COUNT];
int sensorAbyssMax[FLOOR_SENSORS_COUNT];

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
// swiped sensors should show 15, fixed sensors 18 cm
floorSensorDefinition floorSensorDefinitions[FLOOR_SENSORS_COUNT]{
//	  sensorId;          sensorName[20];    TYPE; swipe; installed; sensorPin; servoId
	{ FRONT_LEFT,        "front left",       A41, true,  true,      A2,         FL},
	{ FRONT_CENTER,      "front center ",    A41, true,  true,      A7,         FC},
	{ FRONT_RIGHT,       "front right",      A41, true,  true,      A3,         FR},
	{ BACK_LEFT,         "back_left",        A21, true,  true,      A8,         BL},
	{ BACK_CENTER,       "back_center",      A21, true,  true,      A12,        BC},
	{ BACK_RIGHT,        "back_right",       A21, true,  true,      A10,        BR},
	{ LEFT_SIDE_FRONT,   "left side front",  A21, false, true,      A0,         -1},
	{ LEFT_SIDE_BACK,    "left side back",   A21, false, true,      A9,         -1},
	{ RIGHT_SIDE_FRONT,  "right side front", A21, false, true,      A1,         -1},
	{ RIGHT_SIDE_BACK,   "right side back ", A21, false, true,      A11,        -1}
};

int floorSensorReferenceDistances[FLOOR_SENSORS_COUNT][NUM_MEASURE_STEPS];

// DEFINITION of global variables
// sensorId with measured closest obstacle
int maxObstacleSensorId;

// sensorId with measured largest abyss
int maxAbyssSensorId;



// distance 021 20..80 cm
// convert infrared sensor values to mm
// mark invalid values as DISTANCE_UNKNOWN
int analogToDistance(ir_sensor sensor, int adc_value)
{
	if (adc_value < 50 || adc_value > 800)
	{
		if (verbose) {
			Serial.print("raw value out of scope (50..800): ");
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
			Serial.print("raw value out of scope (50..800): ");
			Serial.print(adc_value);
			Serial.println();
		}
		return DISTANCE_UNKNOWN;	//not a valid raw value
	}
	
	return 13 / (adc_value * 0.00048828125);
}


////////////////////////////////////////////////////////////////////////
void setupSwipeServos(int dockingSwitch) {
	Serial.println("setup swipe servos");
	int Pin;
	for (int sensorId = 0; sensorId < SWIPE_SERVOS_COUNT; sensorId++) {
	//for (int sensorId = 2; sensorId < 3; sensorId++) {
		Pin = servoDefinitions[sensorId].servoPin;
		if (servoDefinitions[sensorId].installed) pinMode(Pin, OUTPUT);

		// check for startup with activated docking switch
		// in that case do not run the initial check on the swipe servos as we are right 
		// in front of the power station
		if (dockingSwitch == 0) {

			Serial.println("move swipe servo to middle position for checking correct horn position");
			swipeServos[sensorId].attach(servoDefinitions[sensorId].servoPin);
			swipeServos[sensorId].write(90+servoDefinitions[sensorId].servoDegreeOffset);

			Serial.println("now do some repeated distance measures to verify infrared sensors");
			int distRaw;
			int distMm;
			for (int i = 0; i < 20; i++) {
				distRaw = analogRead(floorSensorDefinitions[sensorId].sensorPin);
				distMm = analogToDistance(GP2Y0A21YK, distRaw);
				Serial.print("sensor: "); Serial.print(floorSensorDefinitions[sensorId].sensorName);
				Serial.print(", distRaw: "); Serial.print(distRaw);
				Serial.print(", distMm: "); Serial.print(distMm);
				Serial.println();
				delay(100);
			}
		}
	}

	stopSwipe();

	Serial.println("setup swipe servos done");
}



void attachServo(int servoID) {
	if (!swipeServos[servoID].attached()) {
		swipeServos[servoID].attach(servoDefinitions[servoID].servoPin);
	}
}

/////////////////////////
/////////////////////////
void nextSwipeServoStep() {

	int relAngle;

	nextMeasureStep = currentMeasureStep + swipeDirection;

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

			/*
			Serial.println((String)"I101 currentMeasureStep: " + currentMeasureStep
				+ ", servoId: " + servoId
				+ ", relAngle: " + relAngle
				+ ", nextSwipeServoAngle: " + nextSwipeServoAngle);
			*/

			swipeServos[servoId].write(nextSwipeServoAngle);
		}
	}
	return;
}



///////////////////////////////////////////////////
// check for current data in distance sensor values
///////////////////////////////////////////////////
bool isDataCurrent(int sensorId) {

	unsigned long oldestMeasurement = pow(2, 32) - 1;
	bool current;

	if (floorSensorDefinitions[sensorId].swipe) {
		for (int i = 0; i < NUM_MEASURE_STEPS; i++) {
			oldestMeasurement = min(oldestMeasurement, sensorData[sensorId][i].lastMeasurementTime);
		}
		current = (millis() - oldestMeasurement) < (NUM_MEASURE_STEPS * MIN_MEASURE_CYCLE_DURATION);

		if (verbose && !current) {
			Serial.println((String)"swipe sensor data out of date, oldest: "
				+ (millis() - oldestMeasurement)
				+ ", limit: " + (NUM_MEASURE_STEPS * MIN_MEASURE_CYCLE_DURATION)
				+ ", current: " + (current ? "true" : "false"));
		}
	}
	else {
		current = true;		// non-swiping sensors 
	}
	return current;
}


int calcFloorOffset(int sensorId, int swipeStep) {
	return floorSensorReferenceDistances[sensorId][swipeStep] - sensorData[sensorId][swipeStep].distance;
}


///////////////////////////////////
// find closest obstacle of sensor in all swipe positions
///////////////////////////////////
int maxObstacleSensor(int sensorId) {

	//Serial.println("highest obstacle ");
	int thisOffset;
	int maxObstacle = 0;

	int numMeasureSteps = floorSensorDefinitions[sensorId].swipe ? NUM_MEASURE_STEPS : 1;

	//Serial.print("min value, sensorId: "); Serial.print(sensorId);
	for (int swipeStep = 0; swipeStep < numMeasureSteps; swipeStep++) {
		//Serial.print(", "); Serial.print(sensorData[sensorId][swipeStep].Distance);
		thisOffset = calcFloorOffset(sensorId, swipeStep);
		if (thisOffset > 0 && sensorData[sensorId][swipeStep].distance != DISTANCE_UNKNOWN) {
			maxObstacle = max(maxObstacle, thisOffset);
		}
	}

	if (verbose && maxObstacle > FLOOR_MAX_OBSTACLE) {
		Serial.print(getInfraredSensorName(sensorId));
		for (int swipeStep = 0; swipeStep < numMeasureSteps; swipeStep++) {
			Serial.print(calcFloorOffset(sensorId, swipeStep));
			Serial.print(", ");
		}
		Serial.print(" max obstacle: "); 
		Serial.println(maxObstacle);
	}

return maxObstacle;
}


///////////////////////////////////
// find deeptest abyss of sensor in all swipe positions
///////////////////////////////////
int maxAbyssSensor(int sensorId) {

	//Serial.println("highest obstacle ");
	int thisOffset;
	int maxAbyss = 0;

	int numMeasureSteps = floorSensorDefinitions[sensorId].swipe ? NUM_MEASURE_STEPS : 1;

	//Serial.print("min value, sensorId: "); Serial.print(sensorId);
	for (int swipeStep = 0; swipeStep < numMeasureSteps; swipeStep++) {
		//Serial.print(", "); Serial.print(sensorData[sensorId][sensorId].Distance);
		thisOffset = calcFloorOffset(sensorId, swipeStep);
		if (thisOffset < 0 && sensorData[sensorId][swipeStep].distance != DISTANCE_UNKNOWN) {
			maxAbyss = max(maxAbyss, -thisOffset);
		}
	}

	if (verbose && maxAbyss > FLOOR_MAX_ABYSS) {
		Serial.print(getInfraredSensorName(sensorId));
		for (int swipeStep = 0; swipeStep < numMeasureSteps; swipeStep++) {
			Serial.print(calcFloorOffset(sensorId,swipeStep));
			Serial.print(", ");
		}
		Serial.print(" max abyss: ");
		Serial.println(maxAbyss);
	}
	return maxAbyss;
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
	//Serial.print("sorted: ");
	//for (int sensorId = 0; sensorId < numItems; sensorId++) {
	//	Serial.print(arr[sensorId]); Serial.print(", ");
	//}
	//Serial.println();
}


///////////////////////////////////////////////////////
// calculate average distance from a number of measures
///////////////////////////////////////////////////////
void calcDistanceAverage() {

	for (int sensorId = 0; sensorId < FLOOR_SENSORS_COUNT; sensorId++) {

		// check for swiping servo
		swipeStep = floorSensorDefinitions[sensorId].swipe ? currentMeasureStep : 0;

		if (sensorInvolved[sensorId]) {

			int avg = 0;
			int numValues = 0;
			int sumRaw = 0;
			int raw = 0;
			int range;
			int medianRaw;
			int distMm;

			// check for reasonable sensor range
			if (sensorDataSwipeStep[sensorId].range > MAX_RAW_RANGE) {
				distMm = DISTANCE_UNKNOWN;

				Serial.print("I101: raw measures, sensor: "); 
				Serial.print(floorSensorDefinitions[sensorId].sensorName); 
				Serial.print(", raw: ");
				for (int i = 0; i < NUM_REPEATED_MEASURES; i++) {
					Serial.print(sensorDataSwipeStep[sensorId].raw[i]);
					Serial.print(", ");
				}
				Serial.println();
			} else {

				/* 8.2.20, why do I get unreasonable analog reads? Do not try to hide them
				// leave out lowest and highest values from sorted list
				*/
				for (int i = 1; i < NUM_REPEATED_MEASURES - 2; i++) {
					//sumDistance += sensorDataSwipeStep[sensorId].mm[sensorId];
					raw = sensorDataSwipeStep[sensorId].raw[i];
					sumRaw += raw;
					numValues += 1;
				}

				// average the remaining values
				//sensorData[sensorId][swipeStep].distance = round(sumDistance / numValues);
				//avgRaw = round(sumRaw / numValues);

				// try with median as avg still shows large deviations
				medianRaw = sensorDataSwipeStep[sensorId].raw[int(NUM_REPEATED_MEASURES / 2)];

				// calculate mm from raw average for the sensortype used
				if (floorSensorDefinitions[sensorId].sensorType == A41) {
					distMm = analogToDistanceA41(medianRaw);

					// A41 sensors need some time to adjust, ignore first values after move start
					//if ((millis() - msMoveCmd) < 500) {
					//	distMm = DISTANCE_UNKNOWN;
					//}

				}
				if (floorSensorDefinitions[sensorId].sensorType == A21) {
					distMm = analogToDistance(GP2Y0A21YK, medianRaw);
				}


				// log raw value range for sensor and swipeStep
				Serial.print("I100 sensor: "); Serial.print(floorSensorDefinitions[sensorId].sensorName);
				Serial.print(" , swipeStep: "); Serial.print(swipeStep);
				Serial.print(", swipeDir: "); Serial.print(swipeDirection);
				Serial.print(", rawRange: "); Serial.print(sensorDataSwipeStep[sensorId].range);
				Serial.print(", medianRaw: "); Serial.print(medianRaw);
				Serial.print(", distance [mm]: "); Serial.print(distMm);

				if (sensorToTest > 0 && verbose) {
					Serial.println((String)getInfraredSensorName(sensorId)
						+ ", dir: " + swipeDirection
						+ ", swipeStep: " + swipeStep
						+ ", angle: " + nextSwipeServoAngle
						+ ", sumDist: " + sumRaw
						+ ", numVal: " + numValues
						+ ", min: " + sensorDataSwipeStep[sensorId].raw[0]
						+ ", max: " + sensorDataSwipeStep[sensorId].raw[NUM_REPEATED_MEASURES-1]
						+ ", range: " + sensorDataSwipeStep[sensorId].range
						+ ", median: " + medianRaw);
				}
			}

			// update last measure time for this sensor and measure step
			if (distMm != DISTANCE_UNKNOWN) {
				sensorData[sensorId][swipeStep].distance = distMm;
				sensorData[sensorId][swipeStep].lastMeasurementTime = millis();
			}
		}
	}
}


/////////////////////////////////////////////////////////
// get sensor distance values for current swipe direction
// and for all sensors in the movement direction
/////////////////////////////////////////////////////////
int readDistanceSensorRawValues(MOVEMENT activeCartMovement) {

	int distRaw;

	// for each sensor involved do a number of repeated measures
	for (int m = 0; m < NUM_REPEATED_MEASURES; m++) {

		for (int sensorId = 0; sensorId < FLOOR_SENSORS_COUNT; sensorId++) {

			// for the sensors involved in the cartDirection
			//if (sensorInvolved[sensorId] && sensorDataSwipeStep[sensorId].range > MAX_RAW_RANGE) {
			if (sensorInvolved[sensorId]) {
				swipeStep = floorSensorDefinitions[sensorId].swipe ? currentMeasureStep : 0;

				distRaw = analogRead(floorSensorDefinitions[sensorId].sensorPin);
				sensorDataSwipeStep[sensorId].raw[m] = distRaw;
				
				delay(1);	//activate/adjust to get reasonable analog readings

				// log every measurement in sensor test and verbose mode
				if (sensorToTest == sensorId && verbose) {
					Serial.println((String)getInfraredSensorName(sensorId)
						+ ", swipeDir: " + swipeDirection
						+ ", swipeStep:" + swipeStep
						+ ", analogIn: A" + (floorSensorDefinitions[sensorId].sensorPin - 54)
						+ ", measure: " + m
						+ ", raw: " + distRaw);
				}
			}
		}

		delay(DELAY_BETWEEN_ANALOG_READS);	// set to a minimal value while getting reasonable analog read values
	}

	int maxRange = 0;
	// for each sensor eval the value range of the raw values
	for (int sensorId = 0; sensorId < FLOOR_SENSORS_COUNT; sensorId++) {

		// for the sensors involved in the cartDirection
		if (sensorInvolved[sensorId]) {

			// sort the raw values and set range while leaving out lowest and highest value
			bubbleSort(sensorDataSwipeStep[sensorId].raw, NUM_REPEATED_MEASURES);
			sensorDataSwipeStep[sensorId].range =
				sensorDataSwipeStep[sensorId].raw[NUM_REPEATED_MEASURES - 2] -
				sensorDataSwipeStep[sensorId].raw[1];

			if (sensorDataSwipeStep[sensorId].range >= MAX_RAW_RANGE) {
				Serial.print("I102: distance sensor raw values out of reasonable range (");
				Serial.print(MAX_RAW_RANGE);
				Serial.print("), range: ");
				Serial.print(sensorDataSwipeStep[sensorId].range);
				Serial.print(" sensor: "); Serial.print(floorSensorDefinitions[sensorId].sensorName);
				Serial.println();
			}
			// find the maxRange
			maxRange = max(maxRange, sensorDataSwipeStep[sensorId].range);
		}
	}
	return maxRange;
}

/////////////////////////////////////////////////////////
// based on the collected raw values of the sensors
// calculate the distances
/////////////////////////////////////////////////////////
void evalObstacleOrAbyss(MOVEMENT activeCartMovement) {

	// use measure step data to set the average distance values for sensorId/swipeStep
	calcDistanceAverage();

	// process distance data only for sensors involved in movement
	for (int sensorId = 0; sensorId < FLOOR_SENSORS_COUNT; sensorId++) {
		if (!sensorInvolved[sensorId]) continue;

		// sensor is involved in the move direction
		// swipeStep is either swipe step or 0
		swipeStep = floorSensorDefinitions[sensorId].swipe ? currentMeasureStep : 0;

		// use the calibrated floor distance to find max obstacle or abyss
		int floorOffset;
		if (sensorData[sensorId][swipeStep].distance == DISTANCE_UNKNOWN) {
			floorOffset = DISTANCE_UNKNOWN;
		}
		else {
			floorOffset = floorSensorReferenceDistances[sensorId][swipeStep] - sensorData[sensorId][swipeStep].distance;
			if (floorOffset > sensorObstacleMax[sensorId]) sensorObstacleMax[sensorId] = floorOffset;
			if (-floorOffset > sensorAbyssMax[sensorId]) sensorAbyssMax[sensorId] = -floorOffset;
		}
		if (sensorToTest > 0 && verbose) {		// in sensor test and verbose mode 
			Serial.println((String)getInfraredSensorName(sensorId)
				+ ", swipeStep: " + swipeStep
				+ ", distance: " + sensorData[sensorId][swipeStep].distance
				+ ", reference: " + floorSensorReferenceDistances[sensorId][swipeStep]
				+ ", floorOffset: " + floorOffset
				+ ", max obstacle: " + sensorObstacleMax[sensorId]
				+ ", max abyss: " + sensorAbyssMax[sensorId]);
		}
	}

	// log the floor offset of involved sensors of the cart direction move
	// once for every swipe
	if (activeCartMovement != STOP) {

		for (int sensorId = 0; sensorId < FLOOR_SENSORS_COUNT; sensorId++) {
			if (!sensorInvolved[sensorId]) continue;

			// check for last position in swipe
			if ((nextMeasureStep == NUM_MEASURE_STEPS) 
				|| nextMeasureStep == -1
				|| !floorSensorDefinitions[sensorId].swipe) {

				logFloorOffset(sensorId);

				// in case of a sensor test log the measured distances too
				if (sensorId == sensorToTest) {
					logDistanceValues(sensorId);
				}

				Serial.println((String)"swipe summary: " + getInfraredSensorName(sensorId)
					+ ", sensor obstacle max: " + sensorObstacleMax[sensorId]
					+ ", sensor abyss max: " + sensorAbyssMax[sensorId]);
				sensorObstacleMax[sensorId] = 0;
				sensorAbyssMax[sensorId] = 0;
			}
		}
	}
}


/////////////////////////
/////////////////////////
void logFloorOffset(int sensorId) {

	String msg;
	int thisOffset;

	// check type of sensor, swiping or fixed
	int numMeasureSteps = floorSensorDefinitions[sensorId].swipe ? NUM_MEASURE_STEPS : 1;
		
	// !A1,<sensorId>,<NUM_MEASURE_STEPS>,[<floorOffsetValues>]
	msg = (String)"!A1," + sensorId + "," + numMeasureSteps + ",";

	for (int swipeStep = 0; swipeStep < numMeasureSteps; swipeStep++) {
		thisOffset = floorSensorReferenceDistances[sensorId][swipeStep] - sensorData[sensorId][swipeStep].distance;

		// log the floor offset
		msg += (String)(thisOffset) + ",";
		//Serial.print(thisOffset); Serial.print(",");
	}
	Serial.println(msg);
}

/////////////////////////////
void logDistanceValues(int sensorId) {

	String msg;
	int distance;

	// check type of sensor, swiping or fixed
	int numMeasureSteps = floorSensorDefinitions[sensorId].swipe ? NUM_MEASURE_STEPS : 1;

	// !A1,<sensorId>,<NUM_MEASURE_STEPS>,[<distanceValues>,]
	msg = (String)"!A7," + sensorId + "," + numMeasureSteps + ",";

	// add the measured distances
	for (int swipeStep = 0; swipeStep < numMeasureSteps; swipeStep++) {
		msg += (String) sensorData[sensorId][swipeStep].distance + ",";
	}

	Serial.println(msg);
}


//////////////////////////////////////////////////////////////////////////
// evaluate min/max values of FLOOR DISTANCE SENSORS
//////////////////////////////////////////////////////////////////////////
bool checkFloorSensors(MOVEMENT direction) {

	// reset max values
	int maxObstacleThisSensor = 0;
	int maxAbyssThisSensor = 0;

	// for all involved distance sensors
	for (int sensorId = 0; sensorId < FLOOR_SENSORS_COUNT; sensorId++) {

		if (sensorInvolved[sensorId]) {

			floorSensorDefinition *senDef = &floorSensorDefinitions[sensorId];

			//Serial.print("check sensor distance: "); Serial.print(FLOOR_SENSOR_RANGE_NAMES[range]);
			if (senDef->installed) {

				// for swiping sensors we need data from all swipe directions
				if (senDef->swipe && !isDataCurrent(sensorId)) {
					return false;
				}

				// min/max values of sensor from all swipe directions
				maxObstacleThisSensor = max(maxObstacleThisSensor, maxObstacleSensor(sensorId));
				maxAbyssThisSensor = max(maxAbyssThisSensor, maxAbyssSensor(sensorId));


				// set max obstacle of all sensors
				if (maxObstacleThisSensor > floorObstacleMax) {
					floorObstacleMax = maxObstacleThisSensor;
					floorObstacleMaxId = sensorId;
					if (verbose) {
						Serial.print("new obstacle max: ");
						Serial.print(floorObstacleMax);
						Serial.print(", sensor: ");
						Serial.print(getInfraredSensorName(floorObstacleMaxId));
						Serial.println();
					}
				}

				// set max abyss of all sensors
				if (maxAbyssThisSensor > floorAbyssMax) {
					floorAbyssMaxId = sensorId;
					floorAbyssMax = maxAbyssThisSensor;
					if (verbose) {
						Serial.print("new obstacle max: ");
						Serial.print(floorObstacleMax);
						Serial.print(", sensor: ");
						Serial.print(getInfraredSensorName(floorObstacleMaxId));
						Serial.println();
					}
				}
			}
		}
	}
	return true;
}



void resetServo(int servoID) {

	// move servo to start position (min)
	int offset = servoDefinitions[servoID].servoDegreeOffset;
	swipeServos[servoID].write(swipeServoStartAngle + offset);

	// will cause first move to servo reset position where it is after a stop
	nextMeasureStep = -1;
	swipeDirection = 1;
}


//////////////////////////////////////////
// stop the distance measure swipe
//////////////////////////////////////////
void stopSwipe() {

	for (int servoID = 0; servoID < SWIPE_SERVOS_COUNT; servoID++) {

		resetServo(servoID);
	}

	// allow servos to move to the reset position before detaching them
	delay(100);

	for (int servoID = 0; servoID < SWIPE_SERVOS_COUNT; servoID++) {
		swipeServos[servoID].detach();
	}
	//Serial.println("distance swipe stopped");
}


const char* getInfraredSensorName(int sensorId) {
	return floorSensorDefinitions[sensorId].sensorName;
}


const char* getUltrasonicSensorName(int sensorId) {
	return ultrasonicDistanceSensor[sensorId].sensorName;
}


void loadFloorReferenceDistances() {

	int eepromStartAddr;
	int byteValue;
	int adjustedValue;

	// load saved reference distances from EEPROM
	for (int sensorId = 0; sensorId < FLOOR_SENSORS_COUNT; sensorId++) {
	
		Serial.print("loaded distances "); Serial.print(sensorId); Serial.print(": ");
		int eepromStartAddr = sensorId * NUM_MEASURE_STEPS;

		for (int swipeStep = 0; swipeStep < NUM_MEASURE_STEPS; swipeStep++) {
			byteValue = EEPROM.read(eepromStartAddr + swipeStep);
			adjustedValue = byteValue < 0 ? byteValue + 128 : byteValue;
			floorSensorReferenceDistances[sensorId][swipeStep] = adjustedValue;
			Serial.print(adjustedValue); Serial.print(", ");
		}
		Serial.println();
	}
}


