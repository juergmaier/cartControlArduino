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
long maxMoveMillis;					// max allowed duration for move
int requestedDistance;					// distance to move
bool moveProtected;						// for docking or close encounter with objects suppress collision detection

// actual variables for move
int cartSpeed;
int totalDistanceMoved;			// total distance in current move
int totalAngleMoved;			// total angle moved in current move
int partialDistanceMoved;		// distance since cart last stopped
int accelerationDistance;
int accelerationTime;
long partialMoveMillis;		// total millis of current movement
long totalPartialMoveMillis;		// sum of partialMoveMillis not including movement in progress
SPEED_PHASE speedPhase;
unsigned long partialMoveStartMillis;		
unsigned long moveRequestReceivedMillis;		// millis when move request received
unsigned long blockedMoveMillis;				// millis when cart stopped
bool freeMove;
bool newSensorValuesAvailable;

int imuYawStart = platformImu.getYaw();			// for rotation end and drift detection/correction during move
int prevDriftCheckDistance;
float driftCompensationLeft = 1.0;
float driftCompensationRight = 0.95;
unsigned long lastPositionSentMillis;

// request variables for rotation
int requestedAngle;						// angle to rotate
int accelerationAngle;					// angle used for acceleration to max speed
int angleDoneWhenBlocked;
int distanceDoneWhenBlocked;
bool reducedSpeed = false;				// flag for using reduced speed by long range sensor obstacle/abyss detection

int MINIMAL_CART_SPEED = 50;			// minimal cart speed 50
int MAX_WAIT_FOR_FREE_MOVE = 3000;		// after detecting obstacle/abyss try to finish move 

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
	int servoID;

	// reset and update the servo and sensor list for the new cartDirection
	for (int m = 0; m < SWIPE_SERVOS_COUNT; m++) { servoInvolved[m] = false; }
	//for (int s = 0; s < FLOOR_SENSORS_COUNT; s++) { sensorInvolved[s] = false; }

	if (verbose && sensorInTest > -1) {
		Serial.print(F("sensorInTest: ")); Serial.print(sensorInTest);
	}
	if (verbose) {
		Serial.print(F("sensors involved: "));
		Serial.print(MOVEMENT_NAMES[plannedCartDirection]);
	}

	if (moveProtected) {

		// build the array of the infrared sensors involved in the planned move
		numInvolvedIrSensors = 0;
		
		for (int s = 0; s < MAX_INVOLVED_IR_SENSORS; s++) {

			// for each direction the set of sensors involved is predefined
			sensorId = directionSensorAssignment[plannedCartDirection][s];

			// for a sensor not involved the value FLOOR_SENSORS_COUNT is set in the table
			if (sensorId != IR_SENSORS_COUNT) {

				// for swiping sensors get the servoId used for swiping
				// for static sensors the servoId is defined as -1
				// a sensor might in addition currently be unavailable (not installed)
				servoID = irSensorDefinitions[sensorId].servoID;
				if (servoID > -1) {
					servoInvolved[servoID] = true;
				}
		
				// check for sensor installed, that we are not in a sensor test
				// and if in sensor test that the loop is at the sensor to test
				if (irSensorDefinitions[sensorId].installed) {
					if (sensorInTest == -1 || sensorId == sensorInTest) {
						involvedIrSensors[numInvolvedIrSensors] = sensorId;
						numInvolvedIrSensors++;
						if (verbose) {
							Serial.print(", ");
							Serial.print(getIrSensorName(sensorId));
						}
					}
				}
			}
		}
		if (verbose) Serial.println();

		// for protected forward move include the ultrasonic sensors
		if (plannedCartDirection == FORWARD) {
			Wire.beginTransmission(ULTRASONIC_DISTANCE_DEVICE_I2C_ADDRESS);
			Wire.write(1);
			Wire.endTransmission();
			if (verbose) {
				Serial.println(F("include ultrasonic sensor results to protect move"));
			}
		}
	}
}



/* track cart move time(s)
The cart can be requested to drive a certain distance or rotate a certain angle within
a given active move time.
Cart movement can be blocked by obstacles and abyss.
In case of a blocked move (cart stopped but total allowed move time and/or distance not reached) 
the software tries to continue the partially done move.
This is the place where partial move times and total move time are taken care of
*/
void trackMoveTime(int currentSpeed, int newSpeed) {

	// handle the different cases
	// check for initialize command of for this move
	if (newSpeed == -1) {
		totalPartialMoveMillis = 0;
		partialMoveMillis = 0;
		distanceDoneWhenBlocked = 0;
		angleDoneWhenBlocked = 0;
		return;
	}

	// after cart blocked free room for move detected
	// current cart speed == 0 and new Speed > 0
	if (currentSpeed == 0 && newSpeed > 0) {
		partialMoveStartMillis = millis();
		if (totalPartialMoveMillis > 0) {
			// continuation of partially done move, init partial move variables
			partialMoveMillis = 0;
			Serial.println(F("continue move"));
		}
		else {
			Serial.println(F("start move"));
		}
	}

	// cart is moving, keep track of progress
	if (currentSpeed > 0 && newSpeed > 0) {
		partialMoveMillis = millis() - partialMoveStartMillis;
		Serial.print(F("currentSpeed: ")); Serial.print(currentSpeed);
		Serial.print(F(", newSpeed: ")); Serial.print(newSpeed);
		Serial.print(F(", partialMoveMillis: ")); Serial.print(partialMoveMillis);
		Serial.println();
	}

	// cart requested to stop
	if (currentSpeed > 0 && newSpeed == 0) {
		partialMoveMillis = millis() - partialMoveStartMillis;
		totalPartialMoveMillis += partialMoveMillis;
		partialMoveMillis = 0;
		blockedMoveMillis = millis();

		if (verbose) {
			Serial.print(millis() - moveRequestReceivedMillis); Serial.print(", ");
			Serial.print(F("cart stopped"));
			Serial.print(F(", currentSpeed: ")); Serial.print(currentSpeed);
			Serial.print(F(", newSpeed: ")); Serial.print(newSpeed);
		}
	}
	return;
}


void applyCartSpeed() {
	
	if (cartSpeed < 0) {
		Serial.print(F("something tried to apply a negative speed: ")); Serial.println(cartSpeed);
		return;
	}

	if (cartSpeed > 0) {

		// check for cart speed request in sensor test mode
		if (sensorInTest > -1) {
			Serial.print(F("PREVENTED SPEED SETTING DURING SENSORTEST"));
			Serial.print(cartSpeed);
			Serial.println();
			return;
		}

	}	
	if (verbose) {
		Serial.print(millis() - moveRequestReceivedMillis);
		Serial.print(F(" ms, new speed: "));
		Serial.println(cartSpeed);
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
	if (millis() - _last12VCheckMillis > 60000) {
		int rawValue = analogRead(PIN_A15);
		//_mVolts = int((rawValue / 1024.0) * 5000); // Gets you mV
		_mVolts = int((rawValue / 256.0) * 5000); // Gets you mV
		_last12VCheckMillis = millis();

		// send info to cart control
		Serial.println();
		Serial.print("!B0,");
		Serial.print(_mVolts);
		Serial.println();
	}
}



void setPlannedCartMove(MOVEMENT newCartAction, int newMaxSpeed, int newDistance, int newDuration, bool newMoveProtected) {

	moveRequestReceivedMillis = millis();
	plannedCartMovement = newCartAction;
	requestedMaxSpeed = newMaxSpeed;
	requestedDistance = newDistance;
	maxMoveMillis = newDuration;
	moveProtected = newMoveProtected;
	blockedMoveMillis = millis();
	freeMove = false;
	
	totalDistanceMoved = 0;
	trackMoveTime(0, -1);		// initialize movement time tracking
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
	swipeStepStartMillis = 0;

	// use the encoder counter to measure distance travelled
	noInterrupts();
	wheelPulseCounter = 0;
	interrupts();
}

void setPlannedCartRotation(MOVEMENT newCartAction, int newMaxSpeed, int newAngle, int maxDuration) {

	moveRequestReceivedMillis = millis();
	plannedCartMovement = newCartAction;
	requestedMaxSpeed = newMaxSpeed;
	requestedAngle = newAngle;
	maxMoveMillis = maxDuration;
	moveProtected = false;
	blockedMoveMillis = millis();
	freeMove = false;

	trackMoveTime(0, -1);		// initialize move duration tracking variables
	imuYawStart = platformImu.getYaw();		// for rotation and drift detection/correction
	prevDriftCheckDistance = 0;
	driftCompensationLeft = 1.0;
	driftCompensationRight = 0.95;

	setInvolvedIrSensors(newCartAction, moveProtected);
	swipeStepStartMillis = 0;

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
		Serial.print(F("obstacle detected by infrared sensor, "));
		Serial.print(F("height: ")); Serial.print(obstacleHeight);
		Serial.print(F(" > limit: ")); Serial.print(limit);
		Serial.print(F(", sensor: ")); Serial.print(getIrSensorName(sensorId));
		Serial.println();

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
void updateDistanceMoved() {

	// in rotation we do not move
	if ((activeCartMovement == ROTATE_CLOCKWISE) || (activeCartMovement == ROTATE_COUNTERCLOCK)) {

		totalDistanceMoved = 0;
		return;
	}

	// for moves use the encoder input
	noInterrupts();
	unsigned long counts = wheelPulseCounter;
	interrupts();

	totalDistanceMoved = int(counts / PULS_PER_MM_STRAIGHT);

	if ((activeCartMovement == LEFT) ||
		(activeCartMovement == RIGHT)) {
		totalDistanceMoved = int(counts / PULS_PER_MM_SIDEWAYS);
	}

	if ((activeCartMovement == BACK_DIAG_LEFT) ||
		(activeCartMovement == BACK_DIAG_RIGHT) ||
		(activeCartMovement == FOR_DIAG_LEFT) ||
		(activeCartMovement == FOR_DIAG_RIGHT)) {
		totalDistanceMoved = int(counts / PULS_PER_MM_DIAGONAL);
	}
	partialDistanceMoved = totalDistanceMoved - distanceDoneWhenBlocked;

	if (verbose) {
		Serial.print(millis() - moveRequestReceivedMillis);
		Serial.print(F(", total dist moved: "));
		Serial.print(totalDistanceMoved);
		Serial.print(F(", activeCartMovement: "));
		Serial.print(MOVEMENT_NAMES[activeCartMovement]);
		Serial.print(F(", encoder counts: "));
		Serial.print(counts);
		Serial.println();
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
		moveStatus = STOPPED;
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
		requestedDistance = totalDistanceMoved + finalDockingMoveDistance;

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
		Serial.print(F("stopCart, moveDone: "));
		Serial.print(moveDone);
		Serial.print(F(" reason: "));
		Serial.println(reason);
	}

	int newSpeed = 0;
	trackMoveTime(cartSpeed, newSpeed);
	cartSpeed = newSpeed;

	distanceDoneWhenBlocked = totalDistanceMoved;
	angleDoneWhenBlocked = totalAngleMoved;

	activeCartMovement = STOP;
	sensorInTest = -1;

	if (moveDone) {

		// move done, stop swiping and log distance
		stopSwipe();
		//maxMoveDuration = 0;
		inFinalDockingMove = false;
		//Serial.println("move done");

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
				+ "," + totalDistanceMoved
				+ "," + reason);
		}

		plannedCartMovement = STOP;
	}
	applyCartSpeed();
	tableStop();
}


void handleRotation(bool newSensorValuesAvailable) {

	if (verbose) {
		Serial.println(F("handle rotation"));
	}

	totalAngleMoved = absAngleDiff(platformImu.getYaw(), imuYawStart);
	if (verbose) {
		Serial.print(F("angle moved: "));
		Serial.print(totalAngleMoved);
		Serial.println();
	}

	if (verbose && totalAngleMoved > 0) {
		Serial.print(F("angleMoved: "));
		Serial.print(totalAngleMoved);
		Serial.print(F(", imuYaw: "));
		Serial.print(platformImu.getYaw());
		Serial.print(F(", _startYaw: "));
		Serial.print(imuYawStart);
		Serial.println();

		Serial.print(F("check rotation done, angleMoved: "));
		Serial.print(totalAngleMoved);
		Serial.print(F(", requestedAngle: "));
		Serial.print(requestedAngle);
		Serial.println();
	}

	if (abs(requestedAngle) - totalAngleMoved < 1) {
		stopCart(true, "rotation done");
		int imuYawEnd = platformImu.getYaw();
		Serial.print("!P2,"); Serial.println(imuYawEnd);

		Serial.print(F(" stop rotation, imuYawStart: ")); Serial.print(imuYawStart);
		Serial.print(F(", requested angle: ")); Serial.print(requestedAngle);
		Serial.print(F(", angle moved: ")); Serial.print(totalAngleMoved);
		Serial.print(F(", imuYawEnd: ")); Serial.print(imuYawEnd);
		Serial.println();

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

			// start with minimal speed and speedPhase ACCELERATE
			trackMoveTime(cartSpeed, MINIMAL_CART_SPEED);
			speedPhase = ACCELERATE;
			if (verbose) {
				Serial.print(F("cartSpeed: "));
				Serial.print(cartSpeed);
				Serial.println(F(", speedPhase ACCELERATE set"));
			}
			activeCartMovement = plannedCartMovement;
		}

		// check for acceleration/deceleration phase
		if (speedPhase == ACCELERATE) {

			// increase speed
			if (cartSpeed < requestedMaxSpeed) {
				// use a ramp to set new speed based on distance driven
				// adjust by multiplying totalDistanceMoved with a reducing/increasing factor
				int newSpeed = min(MINIMAL_CART_SPEED + partialDistanceMoved, requestedMaxSpeed);
				trackMoveTime(cartSpeed, newSpeed);
				cartSpeed = newSpeed;

				// keep track of acceleration to be used in deceleration
				accelerationAngle = totalAngleMoved;
				accelerationTime = millis() - partialMoveStartMillis;

				// check for end of acceleration phase
				if (cartSpeed >= requestedMaxSpeed) {
					speedPhase = CRUISE;
					trackMoveTime(cartSpeed, requestedMaxSpeed);
					cartSpeed = requestedMaxSpeed;

					if (verbose) Serial.println(F("speedPhase CRUISE set"));

					if (verbose) {
						Serial.print(F("acceleration done, cartSpeed: "));
						Serial.print(cartSpeed);
						Serial.print(F(", acceleration angle: "));
						Serial.print(accelerationAngle);
						Serial.print(F(", acceleration time: "));
						Serial.print(accelerationTime);
						Serial.println();
					}
				}
			}
		}

		// check for deceleration
		int remainingAngle = absAngleDiff(requestedAngle, totalAngleMoved);
		int remainingTime = maxMoveMillis - totalPartialMoveMillis - (millis() - partialMoveStartMillis);

		if ((remainingAngle <= (1.0 * accelerationAngle))
			|| (remainingTime <= (1.0 * accelerationTime))) {
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
				Serial.print(remainingAngle);
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
	
		if (totalDistanceMoved - prevDriftCheckDistance > 50) {
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

			prevDriftCheckDistance = totalDistanceMoved;

			if (verbose) {
				Serial.print(F("cart position update, distance: "));
				Serial.print(totalDistanceMoved);
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

	updateDistanceMoved();

	// check for total distance done
	if (abs(totalDistanceMoved - requestedDistance) < 5) {
		stopCart(true, "requested distance moved");

		Serial.print(F("stop move, imuYawStart: "));
		Serial.print(imuYawStart);
		Serial.print(F(", requested distance: "));
		Serial.print(requestedDistance);
		Serial.print(F(", distance moved: "));
		Serial.print(totalDistanceMoved);
		Serial.print(F(", imuYawEnd: "));
		Serial.print(platformImu.getYaw());
		Serial.println();

		return;
	}

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

			// start with minimal speed and speedPhase ACCELERATE
			int newSpeed = MINIMAL_CART_SPEED;
			speedPhase = ACCELERATE;
			trackMoveTime(cartSpeed, newSpeed);
			cartSpeed = newSpeed;
			if (verbose) Serial.println(F("speedPhase ACCELERATE set"));
		}

		activeCartMovement = plannedCartMovement;

		// check for speed phase accelerate
		if (speedPhase == ACCELERATE) {

			// increase speed
			if (cartSpeed < requestedMaxSpeed) {
				int newSpeed = cartSpeed + MAX_SPEED_INCREASE;
				trackMoveTime(cartSpeed, newSpeed);
				cartSpeed = newSpeed;

				// keep track of acceleration distance and time to be used in deceleration
				accelerationDistance = totalDistanceMoved - distanceDoneWhenBlocked;
				accelerationTime = millis() - partialMoveStartMillis;

				// check for end of acceleration phase
				if (cartSpeed >= requestedMaxSpeed) {
					speedPhase = CRUISE;
					if (verbose) Serial.println(F("speedPhase CRUISE set"));

					if (verbose) {
						Serial.print(F("acceleration done, cartSpeed: "));
						Serial.print(cartSpeed);
						Serial.print(F(", acceleration distance: "));
						Serial.print(accelerationDistance);
						Serial.print(F(", acceleration time: "));
						Serial.print(accelerationTime);
						Serial.println();
					}
				}
			}
		}

		// check for deceleration based on distance moved (may be during acceleration)
		int remainingDistance = requestedDistance - totalDistanceMoved;
		int remainingTime = maxMoveMillis - totalPartialMoveMillis - (millis() - partialMoveStartMillis);

		if ((remainingDistance <= (1.0 * accelerationDistance))
			|| (remainingTime <= (1.0 * accelerationTime))) {
			if (speedPhase != DECELERATE) {
				speedPhase = DECELERATE;
				if (verbose) Serial.println(F("speedPhase DECELERATE set"));
			}
		}

		if (speedPhase == DECELERATE) {
			int newSpeed = cartSpeed - MAX_SPEED_DECREASE;
			trackMoveTime(cartSpeed, newSpeed);
			cartSpeed = newSpeed;

			if (cartSpeed < MINIMAL_CART_SPEED) {
				cartSpeed = MINIMAL_CART_SPEED;
			}
			if (verbose) {
				Serial.print(F("decelerate remaining distance: "));
				Serial.print(remainingDistance);
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

		//Serial.println("handleCartMovement");

		// check total move time
		int currentTotalMoveMillis = totalPartialMoveMillis + partialMoveMillis;
		if (currentTotalMoveMillis > maxMoveMillis) {

			stopCart(true, "max move duration reached");

			Serial.print(millis() - moveRequestReceivedMillis); Serial.print(F("ms, "));
			Serial.print(F("maxMoveDuration reached, stopping cart after: ")); Serial.print(currentTotalMoveMillis); 			
			Serial.print(F(" ms, totalPartialMoveMillis: ")); Serial.print(totalPartialMoveMillis);
			Serial.print(F(" ms, partialMoveMillis: ")); Serial.print(millis() - partialMoveStartMillis);
			Serial.print(F(" ms, maxMoveMillis: ")); Serial.print(maxMoveMillis);
			Serial.print(F(" ms, distance: ")); Serial.print(totalDistanceMoved);
			Serial.print(F(", imuYaw: ")); Serial.print(platformImu.getYaw());
			Serial.println();
			return;
		}

		// check continuation time after obstacle or abyss detection
		if (cartSpeed == 0) {

			// cart is currently blocked
			//Serial.println("cart blocked");
			if ((plannedCartMovement != STOP) 
				&& (millis() - blockedMoveMillis > MAX_WAIT_FOR_FREE_MOVE)) {

				stopCart(true, "timeout in start or continuation of move");

				Serial.print(F("max wait after blocking reached, distance: "));
				Serial.print(totalDistanceMoved);
				Serial.print(F(", imuYaw: "));
				Serial.println(platformImu.getYaw());
				return;
			}
		}

		// keep track of total move time
		if (activeCartMovement != STOP) {
			if (cartSpeed > 0) {
				currentTotalMoveMillis = totalPartialMoveMillis + ((millis() - partialMoveStartMillis));
				if (verbose) {
					Serial.print(millis() - moveRequestReceivedMillis); Serial.print(", ");
					Serial.print(F("currentMoveMillis: ")); Serial.print(currentTotalMoveMillis);
					Serial.print(F(", maxMoveDuration: ")); Serial.println(maxMoveMillis);
				}
				// verify we get wheel encoder signals
				if ((sensorInTest > -1) && (millis() - partialMoveStartMillis > 500) && (wheelPulseCounter == 0)) {
					stopCart(false, "missing wheel encoder signals");
					return;
				}
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
			Serial.print(",");	Serial.print(totalDistanceMoved);
			Serial.print(",");	Serial.print(activeCartMovement);
		
			Serial.println();
		}
	}
}
