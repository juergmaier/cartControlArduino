// 
// 
// 
#include <Wire.h>
#include <Servo.h>

#include "cartControlArduino.h"
#include "drive.h"
#include "distance.h"
#include "table.h"
#include "bno055.h"

#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"


// max 6 ir-sensors can be needed for the move
static int directionSensorAssignment[MOVEMENT_COUNT][MAX_INVOLVED_IR_SENSORS]{
	/* STOP              */{ IR_SENSORS_COUNT, IR_SENSORS_COUNT, IR_SENSORS_COUNT, IR_SENSORS_COUNT, IR_SENSORS_COUNT, IR_SENSORS_COUNT },
	/* FORWARD           */{ FRONT_LEFT,          FRONT_CENTER,        FRONT_RIGHT,         IR_SENSORS_COUNT, IR_SENSORS_COUNT, IR_SENSORS_COUNT },
	/* FOR_DIAG_RIGHT    */{ FRONT_LEFT,          FRONT_CENTER,        FRONT_RIGHT,         RIGHT_SIDE_FRONT,    RIGHT_SIDE_BACK,     IR_SENSORS_COUNT },
	/* FOR_DIAG_LEFT     */{ FRONT_LEFT,          FRONT_CENTER,        FRONT_RIGHT,         LEFT_SIDE_FRONT,     LEFT_SIDE_BACK,      IR_SENSORS_COUNT },
	/* LEFT              */{ LEFT_SIDE_FRONT,     LEFT_SIDE_BACK,      IR_SENSORS_COUNT, IR_SENSORS_COUNT, IR_SENSORS_COUNT, IR_SENSORS_COUNT },
	/* RIGHT             */{ RIGHT_SIDE_FRONT,    RIGHT_SIDE_BACK,     IR_SENSORS_COUNT, IR_SENSORS_COUNT, IR_SENSORS_COUNT, IR_SENSORS_COUNT },
	/* BACKWARD          */{ BACK_LEFT,           BACK_CENTER,         BACK_RIGHT,          IR_SENSORS_COUNT, IR_SENSORS_COUNT, IR_SENSORS_COUNT },
	/* BACK_DIAG_RIGHT   */{ BACK_LEFT,           BACK_CENTER,         BACK_RIGHT,          RIGHT_SIDE_FRONT,    RIGHT_SIDE_BACK,     IR_SENSORS_COUNT },
	/* BACK_DIAG_LEFT    */{ BACK_LEFT,           BACK_CENTER,         BACK_RIGHT,          LEFT_SIDE_FRONT,     LEFT_SIDE_BACK,      IR_SENSORS_COUNT },
	/* ROTATE_LEFT       */{ LEFT_SIDE_FRONT,     FRONT_LEFT,          FRONT_RIGHT,         RIGHT_SIDE_BACK,     BACK_LEFT,           BACK_RIGHT          },
	/* ROTATE_RIGHT      */{ RIGHT_SIDE_FRONT,    FRONT_LEFT,          FRONT_RIGHT,         LEFT_SIDE_BACK,      BACK_LEFT,           BACK_RIGHT          }
};



uint8_t MAX_SPEED_INCREASE = 8;
uint8_t MAX_SPEED_DECREASE = 12;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *fl = AFMS.getMotor(1);
Adafruit_DCMotor *fr = AFMS.getMotor(2);
Adafruit_DCMotor *br = AFMS.getMotor(3);
Adafruit_DCMotor *bl = AFMS.getMotor(4);

boolean inFinalDockingMove = false;

MOVEMENT_STATUS moveStatus = STOPPED;
MOVEMENT_STATUS prevMoveStatus = STOPPED;

MOVEMENT activeCartMovement;

// request variables for move
MOVEMENT plannedCartMovement;
int requestedMaxSpeed;					// allowed max speed

bool moveProtected;						// for docking or close encounter with objects suppress collision detection

// request variables for rotation
int requestedAngle;						// angle to rotate
//int accelerationAngle;					// angle used for acceleration to max speed
//int angleDoneWhenBlocked;
//int distanceDoneWhenBlocked;
bool proceedWithReducedSpeed = false;	// flag for using reduced speed by long range sensor obstacle/abyss detection

// actual variables for move
int cartSpeed;
SPEED_PHASE speedPhase;
MOVE_TYPE moveType;

// a move can be interrupted by detecting obstacles or abyss
// the cart will try to finish the requested distance/angle move within the max time allowed for the move
// the distance/time moved is accumulated with any early stop in the move
// so we have a current partial move and the sum of the done partial moves 
// for current partial move keep record of its acceleration distance and time
// this allows for a deceleration of the cart analoguos to the acceleration
unsigned long partialMoveStartMillis;	// start of current partial move	
int requestedDistance;					// distance to move
int sumDonePartialDistances;			// distance in previous partial moves
int partialMoveDistance;				// distance in current partial move
int remainingMoveDistance;

int requestedMoveAngle;					// rel angle
int partialMoveStartAngle;				// abs angle
int partialMoveAngle;					// rel angle rotated in current partial move
int sumDonePartialAngles;				// rel angle rotated in previous partial moves
int remainingMoveAngle;					// rel angle to target

int maxMoveMillis;						// max allowed duration for move (when speed > 0)
int partialMoveMillis;					// millis in current partial move
int sumDonePartialMoveMillis;			// millis in previous partial moves
int remainingMoveMillis;				// 

int accelerationDistance;				// distance in acceleration phase in current partial move
int accelerationAngle;					// angle in acceleration phase in current partial move
int accelerationMillis;		// millis in acceleration mode in current partial move

// millis when move request received, this allows for easier sequence detection in the logs
unsigned long moveRequestReceivedMillis;		
unsigned long blockedMoveMillis;		// millis when cart last stopped, to terminate move when blocked
bool freeMove;							// a flag for no obstacle/abyss detected by the sensors
bool newSensorValuesAvailable;

// cart moves can be non-straight when wheel speed differences or slides occur
// try to adjust the wheel speeds to get a straight move (platform imu will show drifts)
int imuYawStart = platformImu.getYaw();			// for rotation end and drift detection/correction during move
int prevDriftCheckDistance;
float driftCompensationLeft = 1.0;
float driftCompensationRight = 0.95;
unsigned long lastPositionSentMillis;


int MINIMAL_CART_SPEED = 50;			// minimal cart speed 50
int MAX_WAIT_FOR_FREE_MOVE = 6000;		// after detecting obstacle/abyss try to finish move 

int moves[11][4]{
	/* STOP             */{ RELEASE,  RELEASE,  RELEASE,  RELEASE  },
	/* FORWARD          */{ FORWARD,  FORWARD,  FORWARD,  FORWARD  },
	/* FOR_DIAG_RIGHT   */{ RELEASE,  FORWARD,  FORWARD,  RELEASE  },
	/* FOR_DIAG_LEFT    */{ FORWARD,  RELEASE,  RELEASE,  FORWARD  },
	/* LEFT             */{ FORWARD,  BACKWARD, BACKWARD, FORWARD  },
	/* RIGHT            */{ BACKWARD, FORWARD,  FORWARD,  BACKWARD },
	/* BACKWARD         */{ BACKWARD, BACKWARD, BACKWARD, BACKWARD },
	/* BACK_DIAG_RIGHT  */{ BACKWARD, RELEASE,  RELEASE,  BACKWARD },
	/* BACK_DIAG_LEFT   */{ RELEASE,  BACKWARD, BACKWARD, RELEASE  },
	/* ROTATE_LEFT      */{ FORWARD,  BACKWARD, FORWARD,  BACKWARD },
	/* ROTATE_RIGHT     */{ BACKWARD, FORWARD,  BACKWARD, FORWARD  }
};



void setupFahren() {

	Serial.println(F("initialize cart wheel motor control"));			// Adafruit Motor Shield
	AFMS.begin();
	Serial.println(F("initialize cart wheel motor control done")); 	// Adafruit Motor Shield
}




int absAngleDiff(int a, int b) {
	int diff = (a % 360) - (b % 360) + 360;		// arduino can not modulo with neg numbers
	return abs((diff + 180) % 360 - 180) % 360;	
}


void setInvolvedIrSensors(MOVEMENT plannedCartDirection, bool moveProtected) {

	int sensorId;
	int servoId;

	// reset and update the servo and sensor list for the new cartDirection
	for (int m = 0; m < SWIPE_SERVOS_COUNT; m++) { servoInvolved[m] = false; }

	if (moveType == SENSORTEST) {
		prt("sensorInTest: "); prl(sensorInTest);
	}
	if (verbose) {
		prt(", sensors involved: "); pr(MOVEMENT_NAMES[plannedCartDirection]);
	}

	if (moveProtected) {

		// build the array of the infrared sensors involved in the planned move
		numInvolvedIrSensors = 0;
		bool initialAnalogReadDone = false;

		for (int s = 0; s < MAX_INVOLVED_IR_SENSORS; s++) {

			// for each direction the set of sensors involved is predefined
			sensorId = directionSensorAssignment[plannedCartDirection][s];

			// for a sensor not involved the value IR_SENSORS_COUNT is set in the table
			if (sensorId != IR_SENSORS_COUNT) {

				// for swiping sensors get the servoId used for swiping
				// for static sensors the servoId is defined as -1
				// a sensor might in addition currently be unavailable (not installed)
				servoId = irSensorDefinitions[sensorId].servoId;
				if (servoId > -1) {
					servoInvolved[servoId] = true;
				}
		
				// check for sensor installed, that we are not in a sensor test
				// and if in sensor test that the loop is at the sensor to test
				if (irSensorDefinitions[sensorId].installed) {
					if (moveType != SENSORTEST || sensorId == sensorInTest) {
						involvedIrSensors[numInvolvedIrSensors] = sensorId;

						// for the first involved sensor run a few analog reads to adjust to distance
						for (int i = 0; i < IR_MIN_READS_TO_ADJUST; i++){
							analogRead(irSensorDefinitions[sensorId].sensorPin);
							delayMicroseconds(600);
						}

						numInvolvedIrSensors++;
						if (verbose) {
							Serial.print(", ");
							Serial.print(getIrSensorName(sensorId));
						}
					}
				}
			}
		}
		if (verbose) prl();

		// for protected forward move include the ultrasonic sensors
		if (plannedCartDirection == FORWARD && moveType == STRAIGHT) {
			Wire.beginTransmission(ULTRASONIC_DISTANCE_DEVICE_I2C_ADDRESS);
			Wire.write(1);
			Wire.endTransmission();
			if (verbose) {
				Serial.println(F("include ultrasonic sensor results to protect move"));
			}
		}
	}
}


void applyCartSpeed() {
	
	if (cartSpeed < 0) {
		Serial.print(F("something tried to apply a negative speed: ")); Serial.println(cartSpeed);
		return;
	}

	if (cartSpeed > 0) {

		// check for cart speed request in sensor test mode
		if (moveType == SENSORTEST) {
			prt("PREVENTED SPEED SETTING DURING SENSORTEST"); prl(cartSpeed);
			return;
		}

	}	
	if (verbose) {
		Serial.print(millis() - moveRequestReceivedMillis);
		prt(" ms, new speed: "); prl(cartSpeed);
	}

	fl->setSpeed(round(cartSpeed * _speedUnifyer[MOTOR_FRONT_LEFT] / 100) * driftCompensationLeft);
	fr->setSpeed(round(cartSpeed * _speedUnifyer[MOTOR_FRONT_RIGHT] / 100) * driftCompensationRight);
	br->setSpeed(round(cartSpeed * _speedUnifyer[MOTOR_BACK_RIGHT] / 100) * driftCompensationRight);
	bl->setSpeed(round(cartSpeed * _speedUnifyer[MOTOR_BACK_LEFT] / 100) * driftCompensationLeft);

	if (plannedCartMovement == STOP) {
		// cut 12V motor power
		digitalWrite(PIN_DRIVE_POWER, SWITCH_OFF);
	}
	else {
		// provide 12 V motor power
		digitalWrite(PIN_DRIVE_POWER, SWITCH_ON);
	}

	// set wheel motor action based on movement
	// with the help of the movement/direction matrix
	fl->run(moves[activeCartMovement][MOTOR_FRONT_LEFT]);
	fr->run(moves[activeCartMovement][MOTOR_FRONT_RIGHT]);
	br->run(moves[activeCartMovement][MOTOR_BACK_RIGHT]);
	bl->run(moves[activeCartMovement][MOTOR_BACK_LEFT]);


	// check for 12V Voltage about every Minute while cart is moving
	if (millis() - last12VCheckMillis > 60000) {
		int rawValue = analogRead(PIN_A15);
		int mVolts = int((rawValue / 256.0) * 5000); // Gets you mV
		last12VCheckMillis = millis();

		// send info to cart control
		Serial.println();
		Serial.print("!B0,");
		Serial.print(mVolts);
		Serial.println();
	}
}



void setPlannedCartMove(MOVEMENT newCartAction, int newMaxSpeed, int newDistance, int newDuration, bool newMoveProtected) {

	moveRequestReceivedMillis = millis();
	plannedCartMovement = newCartAction;
	activeCartMovement = STOP;
	requestedMaxSpeed = newMaxSpeed;
	requestedDistance = newDistance;
	maxMoveMillis = newDuration;
	moveProtected = newMoveProtected;
	blockedMoveMillis = millis();
	freeMove = false;
	
	sumDonePartialDistances = 0;
	partialMoveDistance = 0;
	sumDonePartialAngles = 0;
	partialMoveAngle = 0;
	sumDonePartialMoveMillis = 0;
	partialMoveMillis = 0;
	imuYawStart = platformImu.getYaw();		// for rotation and drift detection/correction
	prevDriftCheckDistance = 0;
	driftCompensationLeft = 1.0;
	driftCompensationRight = 0.95;

	Serial.print(F("plannedCartMove: ")); 
	Serial.print(plannedCartMovement); Serial.print("-"); Serial.print(MOVEMENT_NAMES[plannedCartMovement]);
	Serial.print(F(", speed: ")); Serial.print(requestedMaxSpeed);
	Serial.print(F(", dist: ")); Serial.print(requestedDistance);
	Serial.print(F(" mm, maxMoveDuration: ")); Serial.print(maxMoveMillis);
	Serial.print(F(" ms, moveProtected: ")); Serial.print(moveProtected ? "true" : "false");
	Serial.println();

	setInvolvedIrSensors(newCartAction, moveProtected);
	swipeStepStartMillis = millis();

}

void setPlannedCartRotation(MOVEMENT newCartAction, int newMaxSpeed, int newAngle, int maxDuration) {

	moveRequestReceivedMillis = millis();
	plannedCartMovement = newCartAction;
	activeCartMovement = STOP;
	requestedMaxSpeed = newMaxSpeed;
	requestedAngle = newAngle;
	maxMoveMillis = maxDuration;
	moveProtected = true;
	blockedMoveMillis = millis();
	freeMove = false;

	sumDonePartialDistances = 0;
	partialMoveDistance = 0;
	sumDonePartialAngles = 0;
	partialMoveAngle = 0;
	sumDonePartialMoveMillis = 0;
	partialMoveMillis = 0;
	imuYawStart = platformImu.getYaw();		// for rotation and drift detection/correction

	imuYawStart = platformImu.getYaw();		// for rotation and drift detection/correction
	prevDriftCheckDistance = 0;
	driftCompensationLeft = 1.0;
	driftCompensationRight = 0.95;

	setInvolvedIrSensors(newCartAction, moveProtected);
	swipeStepStartMillis = millis();

	if (true) {
		Serial.print(F("cartRotation: "));
		Serial.print(MOVEMENT_NAMES[plannedCartMovement]);
		Serial.print(F(", speed: "));
		Serial.print(requestedMaxSpeed);
		Serial.print(F(", requestedAngle: "));
		Serial.print(requestedAngle);
		Serial.print(F(" mm, maxMoveDuration: "));
		Serial.print(maxMoveMillis);
		Serial.println();
	}
}


void logIrObstacle(int obstacleHeight, int limit, int sensorId) {

	if (moveStatus != OBSTACLE) {		// do not repeat messages

		// log message of obstacle (free text)
		Serial.print(F("obstacle detected by ir sensor, "));
		Serial.print(F("height: ")); Serial.print(obstacleHeight);
		Serial.print(F(" > limit: ")); Serial.print(limit);
		Serial.print(F(", sensor: ")); Serial.print(getIrSensorName(sensorId));
		Serial.println();

		prt("dist: ");
		for (int step = 0; step < NUM_MEASURE_STEPS; step++) {
			pr(irSensorStepData[sensorId][step].distMm);prt(", ");
		}
		prl();

		// message to cart control (structured)
		Serial.print("!S1,"); Serial.print(obstacleHeight);
		Serial.print(","); Serial.print(limit);
		Serial.print(","); Serial.print(sensorId);
		Serial.println();

		moveStatus = OBSTACLE;
		stopCart(false, "obstacle detected");
	}
}


void logIrAbyss(int abyssDepth, int limit, int sensorId) {

	if (moveStatus != ABYSS) {		// do not repeat messages

		// log message of abyss (free text)
		Serial.print(F("abyss detected by infrared sensor, "));
		Serial.print(F("depth: ")); Serial.print(abyssDepth);
		Serial.print(F(" > limit: ")); Serial.print(limit);
		Serial.print(F(", sensor: ")); Serial.print(getIrSensorName(sensorId));
		Serial.println();

		prt("dist: ");
		for (int step = 0; step < NUM_MEASURE_STEPS; step++) {
			pr(irSensorStepData[sensorId][step].distMm);prt(", ");
		}
		prl();

		// message to cart control (structured)
		Serial.print("!S2,"); Serial.print(abyssDepth);
		Serial.print(","); Serial.print(limit);
		Serial.print(","); Serial.print(sensorId);
		Serial.println();

		moveStatus = ABYSS;
		stopCart(false, "abyss detected");
	}

	// loop over involved sensors
	//for (int item = 0; item < numInvolvedIrSensors; item++) {
	//	logFloorOffset(involvedIrSensors[item]);		// log current floor offsets with abyss detection
	//}

}


void logUsObstacle(int obstacleDistance, int limit, int sensorId) {

	if (moveStatus != OBSTACLE) {		// do not repeat messages

		Serial.print(F("obstacle detected by ultrasonic sensor, "));
		Serial.print(F("distance: ")); Serial.print(obstacleDistance);
		Serial.print(F(" > limit: ")); Serial.print(limit);
		Serial.print(F(", sensor: ")); Serial.print(sensorId);

		// message to cart control (structured)
		Serial.print("!S2,"); Serial.print(obstacleDistance);
		Serial.print(","); Serial.print(limit);
		Serial.print(","); Serial.print(sensorId);
		Serial.println();

		moveStatus = OBSTACLE;
		stopCart(false, "obstacle detected");
	}
}


// calculate total distance driven
void updateDistanceAngleMillisMoved() {

	partialMoveMillis = millis() - partialMoveStartMillis;
	remainingMoveMillis = maxMoveMillis - sumDonePartialMoveMillis - partialMoveMillis;

	prt("updateDist.. partialMoveMillis: "); pr(partialMoveMillis);
	prt(", sumDonePartialMoveMillis: "); pr(sumDonePartialMoveMillis);
	prl();

	// for distance use the encoder input ...
	noInterrupts();
	unsigned long counts = wheelPulseCounter;
	interrupts();

	if ((activeCartMovement == FORWARD) ||
		(activeCartMovement == BACKWARD)) {
		partialMoveDistance = int(counts / PULS_PER_MM_STRAIGHT);
	}

	// ... and calc dist based on direction of move
	if ((activeCartMovement == LEFT) ||
		(activeCartMovement == RIGHT)) {
		partialMoveDistance = int(counts / PULS_PER_MM_SIDEWAYS);
	}

	if ((activeCartMovement == BACK_DIAG_LEFT) ||
		(activeCartMovement == BACK_DIAG_RIGHT) ||
		(activeCartMovement == FOR_DIAG_LEFT) ||
		(activeCartMovement == FOR_DIAG_RIGHT)) {
		partialMoveDistance = int(counts / PULS_PER_MM_DIAGONAL);
	}
	remainingMoveDistance = requestedDistance - sumDonePartialDistances - partialMoveDistance;

	if (verbose) {
		pr(millis() - moveRequestReceivedMillis);
		prt(", encoder counts: ");	Serial.print(counts);
		prt(", activeCartMovement: "); pr(MOVEMENT_NAMES[activeCartMovement]);
		prt(", partial dist moved: "); pr(partialMoveDistance);		
		prl();
	}

	// for angle use the platform imu and calc abs angle difference
	partialMoveAngle = absAngleDiff(partialMoveStartAngle, platformImu.getYaw());
	remainingMoveAngle = requestedMoveAngle - sumDonePartialAngles - partialMoveAngle;

	if (speedPhase == ACCELERATE) {
		accelerationDistance = partialMoveDistance;
		accelerationAngle = partialMoveAngle;
		accelerationMillis = millis() - partialMoveStartMillis;
	}

}


int checkUltrasonicDistances() {

	// test for obstacle seen by the ultrasonic front sensors
	int numBytes = Wire.requestFrom(ULTRASONIC_DISTANCE_DEVICE_I2C_ADDRESS, 1 + ULTRASONIC_DISTANCE_SENSORS_COUNT);

	//Serial.print(F("requestFrom, numBytes; "); Serial.println(numBytes);

	// first byte is validity mask
	if (Wire.available()) ultrasonicDistanceSensorValidity = Wire.read();
	//Serial.print(F("usSensor validity: "); Serial.print(ultrasonicDistanceSensorValidity, BIN); Serial.print(F(", distances: ");

	int usSensorId = 0;
	int obstacleSensor = -1;
	int minObstacleDistance = 99;
	if (verbose) Serial.print("!F4,");
	while (Wire.available()) {
		ultrasonicDistanceSensorValues[usSensorId] = Wire.read();
		if (verbose) {
			Serial.print(ultrasonicDistanceSensorValues[usSensorId]); Serial.print(", ");
		}
		if (ultrasonicDistanceSensorValues[usSensorId] > 0 
			&& ultrasonicDistanceSensorValues[usSensorId] < minObstacleDistance){
			minObstacleDistance = ultrasonicDistanceSensorValues[usSensorId];
	  		obstacleSensor = usSensorId;
  		}
		usSensorId++;
	}
	Serial.println();

	// send values to cartControl
	Serial.print("!F4,");
	for (usSensorId = 0; usSensorId < ULTRASONIC_DISTANCE_SENSORS_COUNT; usSensorId++) {
		Serial.print(ultrasonicDistanceSensorValues[usSensorId]); Serial.print(",");
	}
	Serial.println();
	return minObstacleDistance;
}


// find out about obstacles or abysses
bool freeMoveAvailable(MOVEMENT moveDirection) {

	prevMoveStatus = moveStatus;

	if (verbose) {
		Serial.print(F("prevMoveStatus: "));
		Serial.println(MOVEMENT_STATUS_NAMES[prevMoveStatus]);
	}

	// assume free move available
	moveStatus = MOVING;

	// if in sensor test mode do not check for out of data, obstacle or abyss and continue measuring
	if (sensorInTest > -1) {
		if (verbose) Serial.println(F("in sensor test mode, no checking of obstacle or abyss with floor sensors"));
		return true;
	}

	// check for current irSensor distance data
	if (!isIrSensorDataCurrent()) {
		if (cartSpeed > 0) {
			stopCart(false, "no current data from ir sensors");
		}
		return false;
	}

	// test for OBSTACLE detected by infrared sensors
	if (irSensorObstacleMaxValue > FLOOR_MAX_OBSTACLE) {
		logIrObstacle(irSensorObstacleMaxValue, FLOOR_MAX_OBSTACLE, irSensorObstacleMaxSensor);
		return false;
	}

	// test for ABYSS detected by infrared sensors
	if (irSensorAbyssMaxValue > FLOOR_MAX_ABYSS) {
		logIrObstacle(irSensorAbyssMaxValue, FLOOR_MAX_ABYSS, irSensorAbyssMaxSensor);
		return false;
	}

	// in FORWARD move check ultrasonic distances
	if (moveDirection == FORWARD && moveProtected) {
		int minUsObstacleDistance = checkUltrasonicDistances();

		// eval sensor with closest obstacle
		int closestObstacle = 99;
		int usSensor = 0;
		for (int usSensorId = 0; usSensorId < ULTRASONIC_DISTANCE_SENSORS_COUNT; usSensorId++) {
			if (ultrasonicDistanceSensorValues[usSensorId] < closestObstacle) {
				closestObstacle = ultrasonicDistanceSensorValues[usSensorId];
				usSensor = usSensorId;
			}
		}

		if (minUsObstacleDistance < 30) {
			logUsObstacle(minUsObstacleDistance, 30, usSensor);
			return false;
		}
	}



	// test for docking switch
	if (activeCartMovement == FORWARD 
		&& digitalRead(DOCKING_SWITCH_PIN)
		&& inFinalDockingMove == false) {

		// reset distance to drive into dock to configured value
		inFinalDockingMove = true;
		requestedDistance = sumDonePartialDistances + finalDockingMoveDistance;

		Serial.print(F("docking switch activated, continue FORWARD for "));
		Serial.print(finalDockingMoveDistance);
		Serial.println();
		return true;
	}
		
	// check for free move after detecting obstacle or abyss
	if ((prevMoveStatus == OBSTACLE) || (prevMoveStatus == ABYSS)) {

		// inform cartControl
		Serial.print("!S0");
		Serial.println();

		if (verbose) {
			Serial.println(F("continue move after stop"));
		}
	}
	return true;	// move allowed
}


void stopCart(bool moveDone, String reason) {

	if (verbose) {
		prt("stopCart, moveDone: "); pr(moveDone);
		prt(" reason: "); pr(reason);
		prl();
	}

	if (cartSpeed > 0) {

		// stop the wheel motors
		cartSpeed = 0;
		applyCartSpeed();

		sumDonePartialDistances += partialMoveDistance;
		partialMoveDistance = 0;
		sumDonePartialAngles += partialMoveAngle;
		partialMoveAngle = 0;
		sumDonePartialMoveMillis += partialMoveMillis;
		partialMoveMillis = 0;
		blockedMoveMillis = millis();

	}

	activeCartMovement = STOP;
	moveStatus = STOPPED;

	if (moveDone) {

		// move done, stop swiping and log distance
		stopSwipe();
		inFinalDockingMove = false;

		// stop measuring ultrasonic front distances
		Wire.beginTransmission(ULTRASONIC_DISTANCE_DEVICE_I2C_ADDRESS);
		Wire.write(2);
		Wire.endTransmission();

		// inform cartControl of stopped cart and send 
		// - current orientation
		// - distance travelled (based on odometer)
		// - reason for stop
		if (activeCartMovement == ROTATE_CLOCKWISE || activeCartMovement == ROTATE_COUNTERCLOCK) {
			Serial.println((String)"!S4,"
				+ int(platformImu.getYaw())
				+ "," + reason);
		}
		else {
			Serial.println((String)"!S5,"
				+ int(platformImu.getYaw())
				+ "," + sumDonePartialDistances
				+ "," + reason);
		}

		plannedCartMovement = STOP;
	}
	applyCartSpeed();
	tableStop();
}


void handleRotation(bool newSensorValuesAvailable) {

	if (verbose) {
		prtl("handle rotation");
	}

	//totalAngleMoved = absAngleDiff(platformImu.getYaw(), imuYawStart);
	//if (verbose) {
	//	Serial.print(F("angle moved: "));
	//	Serial.print(totalAngleMoved);
	//	Serial.println();
	//}

	if (verbose && remainingMoveAngle > 0) {
		prt("angleMoved: "); pr(sumDonePartialAngles + partialMoveAngle);
		prt(", imuYaw: "); pr(platformImu.getYaw());
		prt(", _startYaw: "); pr(imuYawStart);
		prl();
	}

	if (abs(requestedAngle) - sumDonePartialAngles - partialMoveAngle < 1) {
		stopCart(true, "rotation done");
		int imuYawEnd = platformImu.getYaw();
		Serial.print("!P2,"); Serial.println(imuYawEnd);

		prt(" stop rotation, imuYawStart: "); pr(imuYawStart);
		prt(", requested angle: "); pr(requestedAngle);
		prt(", angle moved: "); pr(sumDonePartialAngles);
		prt(", imuYawEnd: "); pr(imuYawEnd);
		prl();

		return;
	}


	/////////////////////////////////////////////////////////
	// check for obstacles/abyss in planned cart move
	// rotations are always protected against obstacles
	/////////////////////////////////////////////////////////
	if (newSensorValuesAvailable) {
		freeMove = freeMoveAvailable(plannedCartMovement);
	}
	if (freeMove) {

		// check for cart currently stopped
		if (cartSpeed == 0) {

			// start new partial move
			//partialMoveDistance = 0;
			//partialMoveAngle = 0;
			//partialMoveMillis = 0;
			partialMoveStartMillis = millis();
			
			// start with minimal speed and speedPhase ACCELERATE
			//trackMoveTime(cartSpeed, MINIMAL_CART_SPEED);
			speedPhase = ACCELERATE;
			cartSpeed = MINIMAL_CART_SPEED;
			if (verbose) {
				prt("cartSpeed: "); pr(cartSpeed);
				prtl(", speedPhase ACCELERATE set");
			}
			activeCartMovement = plannedCartMovement;
		}

		// check for acceleration/deceleration phase in rotation
		if (speedPhase == ACCELERATE) {

			// increase speed
			if (cartSpeed < requestedMaxSpeed) {
				// use a ramp to set new speed based on distance driven
				// adjust by multiplying totalDistanceMoved with a reducing/increasing factor
				int newSpeed = min(MINIMAL_CART_SPEED + partialMoveDistance, requestedMaxSpeed);
				//trackMoveTime(cartSpeed, newSpeed);
				cartSpeed = newSpeed;

				// check for end of acceleration phase
				if (cartSpeed >= requestedMaxSpeed) {
					speedPhase = CRUISE;
					//trackMoveTime(cartSpeed, requestedMaxSpeed);
					cartSpeed = requestedMaxSpeed;

					if (verbose) prtl("speedPhase CRUISE set");

					if (verbose) {
						prt("acceleration done, cartSpeed: "); pr(cartSpeed);
						prt(", acceleration angle: "); pr(accelerationAngle);
						prt(", acceleration time: "); pr(accelerationMillis);
						prl();
					}
				}
			}
		}

		// check for deceleration in Rotation
		if ((remainingMoveAngle <= (1.0 * accelerationAngle))
			|| (remainingMoveMillis <= (1.0 * accelerationMillis))) {
			if (speedPhase != DECELERATE) {
				speedPhase = DECELERATE;
				if (verbose) Serial.println(F("speedPhase DECELERATE set"));
			}
		}
		if (speedPhase == DECELERATE) {

			cartSpeed -= MAX_SPEED_DECREASE;
			if (cartSpeed < MINIMAL_CART_SPEED) {
				cartSpeed = MINIMAL_CART_SPEED;
			}
			if (verbose) {
				Serial.print(F("decelerate remaining angle: "));
				Serial.print(remainingMoveAngle);
				Serial.print(F(", cartSpeed: "));
				Serial.print(cartSpeed);
				Serial.println();
			}
		}

		activeCartMovement = plannedCartMovement;
		applyCartSpeed();
	}
}


void handleDrift() {

	// with the implementation of individual wheel speed to avoid
	// drift in rotation we introduced drift when moving straight :(
	// in forward move check for changed orientation and reduce
	// left or right wheel speed
	if (activeCartMovement == FORWARD) {
	
		if (partialMoveDistance - prevDriftCheckDistance > 50) {
			// check for full circle crossing
			int offsetImu = 0;
			int offsetStart = 0;
			if (platformImu.getYaw() - imuYawStart > 180) {
				offsetStart = 360;
			}
			if (platformImu.getYaw() - imuYawStart < -180) {
				offsetImu = 360;
			}
			if (((platformImu.getYaw() + offsetImu) - (imuYawStart + offsetStart)) > 0) {
				// drift to the left (anticlock), reduce right motors speed
				if (driftCompensationRight > 0.8) {
					driftCompensationRight *= 0.97;
					driftCompensationLeft = 1.0;
				}
			}
			if (((platformImu.getYaw() + offsetImu) - (imuYawStart + offsetStart)) < 0) {
				// drift to the left (anticlock), reduce right motors speed
				if (driftCompensationLeft > 0.8) {
					driftCompensationLeft *= 0.97;
					driftCompensationRight = 1.0;
				}
			}
			if (platformImu.getYaw() == imuYawStart) {
				driftCompensationLeft = 1.0;
				driftCompensationRight = 0.95;
			}

			prevDriftCheckDistance = partialMoveDistance;

			if (verbose) {
				Serial.print(F("cart position update, distance: "));
				Serial.print(partialMoveDistance);
				Serial.print(F(", startYaw: "));
				Serial.print(imuYawStart);
				Serial.print(F(", imuYaw: "));
				Serial.print(platformImu.getYaw());
				Serial.print(F(", speedLeft: "));
				Serial.print(driftCompensationLeft);
				Serial.print(F(", speedRight: "));
				Serial.print(driftCompensationRight);
				Serial.println();
			}
		}
	}
}


// proceed in driving in one of the directions
void handleMove(bool newSensorValuesAvailable) {

	//updateDistanceAngleMillisMoved();

	// check for total distance done
	if (abs(requestedDistance - sumDonePartialDistances - partialMoveDistance) < 5) {
		stopCart(true, "requested distance moved");

		Serial.print(F("stop move, imuYawStart: "));
		Serial.print(imuYawStart);
		Serial.print(F(", requested distance: "));
		Serial.print(requestedDistance);
		Serial.print(F(", distance moved: "));
		Serial.print(sumDonePartialDistances + partialMoveDistance);
		Serial.print(F(", imuYawEnd: "));
		Serial.print(platformImu.getYaw());
		Serial.println();

		return;
	}

	// check for sensor test, do not move in sensor test mode
	if (moveType == SENSORTEST) return;

	// with new sensor values available check for free move
	if (newSensorValuesAvailable) {
		freeMove = (freeMoveAvailable(plannedCartMovement) || !moveProtected);
	}

	if (freeMove) {

		if (verbose) {
			Serial.print(millis() - moveRequestReceivedMillis);
			Serial.println(F(", free move available"));
		}

		// check for cart currently stopped
		if (cartSpeed == 0) {

			// reset the encoder counter to measure distance travelled
			noInterrupts();
			wheelPulseCounter = 0;
			interrupts();

			// start with minimal speed and speedPhase ACCELERATE
			speedPhase = ACCELERATE;
			cartSpeed = MINIMAL_CART_SPEED;
			if (verbose) prtl("speedPhase ACCELERATE set");
		}

		activeCartMovement = plannedCartMovement;

		// check for speed phase accelerate in move
		if (speedPhase == ACCELERATE) {

			// keep track of acceleration distance and time to be used in deceleration
			accelerationDistance = partialMoveDistance;
			accelerationMillis = millis() - partialMoveStartMillis;

			// increase speed if we have remaining distance and remaining time
			if ((cartSpeed < requestedMaxSpeed) &&
				(remainingMoveDistance > 60) &&
				(remainingMoveMillis > 500)) {
				cartSpeed += MAX_SPEED_INCREASE;
			}
			else {
				speedPhase = CRUISE;
				if (verbose) prtl("speedPhase CRUISE set");
			}
		}

		if (verbose) {
			prt("handleMove, dist requested: "); pr(requestedDistance);
			prt(", moved: "); pr(sumDonePartialDistances);
			prt(", remaining: "); pr(remainingMoveDistance);
			prt(", accelerate: "); pr(accelerationDistance);
			prl();
		}
		// cart decelerate conditions in move:
		if ((remainingMoveDistance < accelerationDistance) 
			|| (remainingMoveMillis < maxMoveMillis - 500)) {
			if (speedPhase != DECELERATE) {
				speedPhase = DECELERATE;
				if (verbose) {
					Serial.print(F("speedPhase DECELERATE set, dist remaining:")); Serial.print(remainingMoveDistance);
					Serial.print(F(", accel: ")); Serial.print(accelerationDistance);
					Serial.print(F(", cartSpeed: ")); Serial.print(cartSpeed);
					Serial.print(F(", rem.Millis: ")); Serial.print(remainingMoveMillis);
					Serial.println();
				}
			}
		}

		if (speedPhase == DECELERATE) {
			//trackMoveTime(cartSpeed, newSpeed);
			cartSpeed -= MAX_SPEED_DECREASE;

			if (cartSpeed < MINIMAL_CART_SPEED) {
				cartSpeed = MINIMAL_CART_SPEED;
			}
			if (verbose) {
				Serial.print(F("decelerate remaining distance: "));
				Serial.print(remainingMoveDistance);
				Serial.print(F(", cartSpeed: "));
				Serial.print(cartSpeed);
				Serial.println();
			}
		}
	
		handleDrift();

		activeCartMovement = plannedCartMovement;
	
		applyCartSpeed();
	}
}


///////////////////////////////////////////////
// main move function called from arduino loop
///////////////////////////////////////////////
void handleCartMovement() {

	if (plannedCartMovement != STOP) {

		if (activeCartMovement != STOP) {

			updateDistanceAngleMillisMoved();

			// check total move time
			if ((sumDonePartialMoveMillis + partialMoveMillis) > maxMoveMillis) {

				stopCart(true, "max move duration reached");

				pr(millis() - moveRequestReceivedMillis); pr("ms, ");
				prt("maxMoveDuration reached, stopping cart after: "); pr(sumDonePartialMoveMillis + partialMoveMillis);
				prt(" ms, maxMoveMillis: "); pr(maxMoveMillis);
				prt(" ms, distance: "); pr(sumDonePartialDistances);
				prt(", imuYaw: "); pr(platformImu.getYaw());
				prl();
				return;
			}

			// check for encoder signals
			if ((millis() - partialMoveStartMillis > 500) && (partialMoveDistance == 0)) {
				stopCart(false, "missing wheel encoder signals");
				return;
			}
		}			


		// check continuation time after obstacle or abyss detection
		if (cartSpeed == 0) {

			// cart is currently blocked
			//Serial.println("cart blocked");
			if ((plannedCartMovement != STOP) 
				&& (millis() - blockedMoveMillis > MAX_WAIT_FOR_FREE_MOVE)) {

				stopCart(true, "timeout in start or continuation of move");

				prt("max wait after blocking reached, distance: "); pr(sumDonePartialDistances);
				prt(", imuYaw: "); pr(platformImu.getYaw());
				prl();
				return;
			}
		}

	
		// for a protected move check the swiping servos state
		if (moveProtected) {

			// the sequence with ir sensors is:
			// check here for sufficient time for the scanning servos to reach a stable target position
			// move the swipe servos to next position (nextMeasureStep) and log the swipeStart time

			//newSensorValuesAvailable = false;
			// check for stable swipe position
			if ((millis() - swipeStepStartMillis) > IR_MIN_WAIT_SWIPE_SERVO) {
				
				// set current measure step to reached step
				currentMeasureStep = nextMeasureStep;
				
				// do a number of distance reads for all involved sensors
				// and store all raw values in irSensorData array
				readIrSensorValues(currentMeasureStep);

				// move swiping servos immediately to next swipe position as this is the
				// time critical part
				nextSwipeServoStep();

				// now process the raw analog distance values measured 
				// for each involved sensor/step evaluate median raw value
				// calc distance and min/max values
				newSensorValuesAvailable = true;
				processNewRawValues(currentMeasureStep);
				logMeasureStepResults();
				setIrSensorsMaxValues();
			}
		}

		// Get current values from imu's
		//headImu.changedBnoSensorData();
		//platformImu.changedBnoSensorData();
		//Serial.print(F("before handle move call, plannedCartMovement: "); Serial.println(plannedCartMovement);

		if (plannedCartMovement == ROTATE_CLOCKWISE || plannedCartMovement == ROTATE_COUNTERCLOCK) {
			handleRotation(newSensorValuesAvailable);
		} else {
			handleMove(newSensorValuesAvailable);
		}
		newSensorValuesAvailable = false;

		// send position update to cartControl
		if ((millis() - lastPositionSentMillis > 200) && (activeCartMovement != STOP)) {
		
			lastPositionSentMillis = millis();
			Serial.println();		// make sure it's a new line
			Serial.print("!P1,");
			Serial.print(int(platformImu.getYaw()));
			Serial.print(",");	Serial.print(sumDonePartialDistances);
			Serial.print(",");	Serial.print(activeCartMovement);
		
			Serial.println();
		}
	}
}
