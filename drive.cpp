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



static int directionSensorAssignment[MOVEMENT_COUNT][MAX_INVOLVED_SENSORS]{
	/* STOP              */{ FLOOR_SENSORS_COUNT, FLOOR_SENSORS_COUNT, FLOOR_SENSORS_COUNT, FLOOR_SENSORS_COUNT, FLOOR_SENSORS_COUNT, FLOOR_SENSORS_COUNT },
	/* FORWARD           */{ FRONT_LEFT,          FRONT_CENTER,        FRONT_RIGHT,         FLOOR_SENSORS_COUNT, FLOOR_SENSORS_COUNT, FLOOR_SENSORS_COUNT },
	/* FOR_DIAG_RIGHT    */{ FRONT_LEFT,          FRONT_CENTER,        FRONT_RIGHT,         RIGHT_SIDE_FRONT,    RIGHT_SIDE_BACK,     FLOOR_SENSORS_COUNT },
	/* FOR_DIAG_LEFT     */{ FRONT_LEFT,          FRONT_CENTER,        FRONT_RIGHT,         LEFT_SIDE_FRONT,     LEFT_SIDE_BACK,      FLOOR_SENSORS_COUNT },
	/* LEFT              */{ LEFT_SIDE_FRONT,     LEFT_SIDE_BACK,      FLOOR_SENSORS_COUNT, FLOOR_SENSORS_COUNT, FLOOR_SENSORS_COUNT, FLOOR_SENSORS_COUNT },
	/* RIGHT             */{ RIGHT_SIDE_FRONT,    RIGHT_SIDE_BACK,     FLOOR_SENSORS_COUNT, FLOOR_SENSORS_COUNT, FLOOR_SENSORS_COUNT, FLOOR_SENSORS_COUNT },
	/* BACKWARD          */{ BACK_LEFT,           BACK_CENTER,         BACK_RIGHT,          FLOOR_SENSORS_COUNT, FLOOR_SENSORS_COUNT, FLOOR_SENSORS_COUNT },
	/* BACK_DIAG_RIGHT   */{ BACK_LEFT,           BACK_CENTER,         BACK_RIGHT,          RIGHT_SIDE_FRONT,    RIGHT_SIDE_BACK,     FLOOR_SENSORS_COUNT },
	/* BACK_DIAG_LEFT    */{ BACK_LEFT,           BACK_CENTER,         BACK_RIGHT,          LEFT_SIDE_FRONT,     LEFT_SIDE_BACK,      FLOOR_SENSORS_COUNT },
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
int maxMoveDuration;					// max allowed duration for move
int requestedDistance;					// distance to move
bool moveProtected;						// for docking or close encounter with objects suppress collision detection

// actual variables for move
int cartSpeed;
int totalDistanceMoved;
int accelerationDistance;
int accelerationTime;
int moveDuration;						// total millis of movement
SPEED_PHASE speedPhase;
unsigned long moveCycleStartMillis;		
unsigned long moveInterruptedMillis;	// time stamp of detected obstacle/abyss
unsigned long msMoveCmd;

int imuYawStart = platformImu.getYaw();				// for rotation end and drift detection/correction during move
int prevDriftCheckDistance;
float driftCompensationLeft = 1.0;
float driftCompensationRight = 0.95;
unsigned long lastPositionSentMillis;

// request variables for rotation
int requestedAngle;						// angle to rotate
int accelerationAngle;					// angle used for acceleration to max speed
int angleAtInterrupt;
int distanceAtInterrupt;
int durationAtInterrupt;
bool reducedSpeed = false;				// flag for using reduced speed by long range sensor obstacle/abyss detection

int MINIMAL_CART_SPEED = 50;			// minimal cart speed 50
int MAX_WAIT_FOR_FREE_MOVE = 4000;		// after detecting obstacle/abyss try to finish move 



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

	Serial.println("initialize cart wheel motor control");			// Adafruit Motor Shield
	AFMS.begin();
	Serial.println("initialize cart wheel motor control done"); 	// Adafruit Motor Shield
}




int absAngleDiff(int a, int b) {
	int diff = (a % 360) - (b % 360) + 360;		// arduino can not modulo with neg numbers
	return abs((diff + 180) % 360 - 180) % 360;	
}


void setInvolvedSensors(MOVEMENT plannedCartDirection, bool moveProtected) {

	int sensorId;
	int servoID;

	// reset and update the servo and sensor list for the new cartDirection
	for (int m = 0; m < SWIPE_SERVOS_COUNT; m++) { servoInvolved[m] = false; }
	for (int s = 0; s < FLOOR_SENSORS_COUNT; s++) { sensorInvolved[s] = false; }

	if (verbose) {
		Serial.print("sensors involved: ");
		Serial.print("sensorToTest: ");
		Serial.print(sensorToTest);
		Serial.print(", plannedCartDirection: ");
		Serial.print(MOVEMENT_NAMES[plannedCartDirection]);
		Serial.print(", sensorsInvolved: ");
	}

	if (moveProtected) {

		// set the infrared sensors involved in the planned move
		for (int s = 0; s < MAX_INVOLVED_SENSORS; s++) {

			sensorId = directionSensorAssignment[plannedCartDirection][s];

			if (sensorId != FLOOR_SENSORS_COUNT) {

				// for activation of swipe servo
				servoID = floorSensorDefinitions[sensorId].servoID;
				servoInvolved[servoID] = true;

				if (floorSensorDefinitions[sensorId].installed) {
					sensorInvolved[sensorId] = (sensorToTest == -1 || sensorId == sensorToTest);
				}
				else sensorInvolved[sensorId] = false;

				if (verbose && sensorInvolved[sensorId]) {
					Serial.print(getInfraredSensorName(sensorId));
					Serial.print(", ");
				}
			}
		}
		// for proteceted forward move start measuring with the ultrasonic sensors
		if (plannedCartDirection == FORWARD) {
			Wire.beginTransmission(ULTRASONIC_DISTANCE_DEVICE_I2C_ADDRESS);
			Wire.write(1);
			Wire.endTransmission();
		}
	}
	if (verbose) Serial.println();
}



void applyCartSpeed() {
	
	if (cartSpeed < 0) {
		Serial.print("something tried to apply a negative speed: "); Serial.println(cartSpeed);
		return;
	}

	if (cartSpeed > 0) {

		// check for cart speed request in sensor test mode
		if (sensorToTest > -1) {
			Serial.print("PREVENTED SPEED SETTING DURING SENSORTEST");
			Serial.print(cartSpeed);
			Serial.println();
			return;
		}

	}	
	if (verbose) {
		Serial.print(millis() - msMoveCmd);
		Serial.print(", new speed: ");
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
		Serial.print("!A8,");
		Serial.print(_mVolts);
		Serial.println();
	}
}


void setPlannedCartMove(MOVEMENT newCartAction, int newMaxSpeed, int newDistance, int newDuration, bool newMoveProtected) {

	msMoveCmd = millis();
	plannedCartMovement = newCartAction;
	requestedMaxSpeed = newMaxSpeed;
	requestedDistance = newDistance;
	maxMoveDuration = newDuration;
	moveProtected = newMoveProtected;

	moveDuration = 0;
	totalDistanceMoved = 0;
	moveInterruptedMillis = 0;
	imuYawStart = platformImu.getYaw();			// for rotation and drift detection/correction
	prevDriftCheckDistance = 0;
	driftCompensationLeft = 1.0;
	driftCompensationRight = 0.95;

	// initialize interrupt values
	// when a move gets interrupted (obstacle/abyss) it tries for some time to finish it
	cartSpeed = 0;
	angleAtInterrupt = 0;
	distanceAtInterrupt = 0;
	durationAtInterrupt = 0;

	setInvolvedSensors(newCartAction, moveProtected);


	// use the encoder counter to measure distance travelled
	noInterrupts();
	wheelPulseCounter = 0;
	interrupts();

	if (verbose) {
		Serial.print("moveDirection: ");
		Serial.print(MOVEMENT_NAMES[plannedCartMovement]);
		Serial.print(", speed: ");
		Serial.print(requestedMaxSpeed);
		Serial.print(", requestedDistance: ");
		Serial.print(requestedDistance);
		Serial.print(" mm, maxMoveDuration: ");
		Serial.print(maxMoveDuration);
		Serial.print(" ms, moveProtected: ");
		Serial.print(moveProtected ? "true" : "false");
		Serial.println();
	}
}

void setPlannedCartRotation(MOVEMENT newCartAction, int newMaxSpeed, int newAngle, int newDuration, bool newMoveProtected) {

	plannedCartMovement = newCartAction;
	requestedMaxSpeed = newMaxSpeed;
	requestedAngle = newAngle;
	maxMoveDuration = newDuration;
	moveProtected = newMoveProtected;
	msMoveCmd = millis();

	moveDuration = 0;
	totalDistanceMoved = 0;
	moveInterruptedMillis = 0;
	imuYawStart = platformImu.getYaw();			// for rotation and drift detection/correction
	prevDriftCheckDistance = 0;
	driftCompensationLeft = 1.0;
	driftCompensationRight = 0.95;

	// initialize interrupt values
	// when a move gets interrupted (obstacle/abyss) it tries for some time to finish it
	cartSpeed = 0;
	angleAtInterrupt = 0;
	//distanceAtInterrupt = 0;
	durationAtInterrupt = 0;

	setInvolvedSensors(newCartAction, moveProtected);


	if (verbose) {
		Serial.print("moveDirection: ");
		Serial.print(MOVEMENT_NAMES[plannedCartMovement]);
		Serial.print(", speed: ");
		Serial.print(requestedMaxSpeed);
		Serial.print(", requestedAngle: ");
		Serial.print(requestedAngle);
		Serial.print(" mm, maxMoveDuration: ");
		Serial.print(maxMoveDuration);
		Serial.print(" ms, moveProtected: ");
		Serial.print(moveProtected ? "true" : "false");
		Serial.println();
	}
}

void stopMotors() {
	cartSpeed = 0;
	applyCartSpeed();
	tableStop();
}


void setMoveInterruptValues() {
	if (moveInterruptedMillis == 0) {
		moveInterruptedMillis = millis();
		angleAtInterrupt = absAngleDiff(platformImu.getYaw(), imuYawStart);
		distanceAtInterrupt = totalDistanceMoved;
		durationAtInterrupt = moveDuration;
		Serial.println("moveInterruptedMillis set");
	}
}


void logAbyss(int abyss, int limit, int sensorId) {

	if (moveStatus != ABYSS) {		// do not repeat messages

		// message to cart control (structured)
		Serial.println((String)"!A3,"
			+ abyss + ","
			+ limit + ","
			+ sensorId);

		// log message of abyss (free text)
		Serial.println((String)"abyss detected, depth: "
			+ abyss + " > limit: " + limit
			+ ", sensor: " + getInfraredSensorName(sensorId)
			+ ", nextSwipeServoAngle: " + nextSwipeServoAngle + " degrees");

		moveStatus = ABYSS;
		stopCart(false, "abyss detected");
		setMoveInterruptValues();
	}

	// loop over all floor sensors
	for (int sensorId = 0; sensorId < FLOOR_SENSORS_COUNT; sensorId++) {

		// check for sensor involved in current move
		if (sensorInvolved[sensorId]) {
			logFloorOffset(sensorId);		// log current floor offsets with abyss detection
		}
	}

}

void logObstacle(DISTANCE_SENSOR_TYPE sensorType, int obstacle, int limit, int sensorId) {

	if (moveStatus != OBSTACLE) {		// do not repeat messages

		// message to cart control (structured)
		Serial.println((String) "!A2," + sensorType 
			+ "," + obstacle
			+ "," + limit
			+ "," + sensorId);

		// log message of obstacle (free text)
		if (sensorType == INFRARED) {
			Serial.println((String)"obstacle detected by infrared sensor, "
				+ "height: " + obstacle
				+ " > limit: " + limit
				+ ", sensor: " + getInfraredSensorName(sensorId)
				+ ", nextSwipeServoAngle: " + nextSwipeServoAngle
				+ " degrees");
		}

		if (sensorType == ULTRASONIC) {
			Serial.println((String)"obstacle detected by ultrasonic sensor, "
				+ "distance: " + obstacle
				+ " > limit: " + limit
				+ ", sensor: " + getUltrasonicSensorName(sensorId));
		}

		moveStatus = OBSTACLE;
		stopCart(false, "obstacle detected");
		setMoveInterruptValues();
	}
	for (int sensorId = 0; sensorId < FLOOR_SENSORS_COUNT; sensorId++) {
		if (sensorInvolved[sensorId]) {
			logFloorOffset(sensorId);		// log current floor offsets with obstacle detection
		}
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

	if (verbose) {
		Serial.print(millis() - msMoveCmd);
		Serial.print(", total dist moved: ");
		Serial.print(totalDistanceMoved);
		Serial.print(", activeCartMovement: ");
		Serial.print(MOVEMENT_NAMES[activeCartMovement]);
		Serial.print(", encoder counts: ");
		Serial.print(counts);
		Serial.println();
	}

}


int checkUltrasonicDistances() {

	// test for obstacle seen by the ultrasonic front sensors
	int numBytes = Wire.requestFrom(ULTRASONIC_DISTANCE_DEVICE_I2C_ADDRESS, 1 + ULTRASONIC_DISTANCE_SENSORS_COUNT);

	//Serial.print("requestFrom, numBytes; "); Serial.println(numBytes);

	// first byte is validity mask
	if (Wire.available()) ultrasonicDistanceSensorValidity = Wire.read();
	//Serial.print("usSensor validity: "); Serial.print(ultrasonicDistanceSensorValidity, BIN); Serial.print(", distances: ");

	int usSensorId = 0;
	int obstacleSensor;
	int minObstacleDistance = 99;
	while (Wire.available()) {
		ultrasonicDistanceSensorValues[usSensorId] = Wire.read();
		//Serial.print(ultrasonicDistanceSensorValues[sensor]); Serial.print(", ");
		if (ultrasonicDistanceSensorValues[usSensorId] > 0 && ultrasonicDistanceSensorValues[usSensorId] < minObstacleDistance){
	  		obstacleSensor = usSensorId;
  		}
		usSensorId++;
	}

	// send values to cartControl
	String msg = "!Af,";
	for (usSensorId = 0; usSensorId < ULTRASONIC_DISTANCE_SENSORS_COUNT; usSensorId++) {
		msg += ultrasonicDistanceSensorValues[usSensorId] + ",";
	}

	return obstacleSensor;
}


bool freeMoveAvailable(MOVEMENT moveDirection) {

	prevMoveStatus = moveStatus;

	if (verbose) {
		Serial.print("prevMoveStatus: ");
		Serial.println(MOVEMENT_STATUS_NAMES[prevMoveStatus]);
	}

	// assume free move available
	moveStatus = MOVING;

	bool floorDistancesValid = checkFloorSensors(moveDirection);


	// check for out of date floor distance values
	if (!floorDistancesValid) {
		moveStatus = STOPPED;
		if (prevMoveStatus != STOPPED) {
			Serial.println("no current values for all involved floor distance sensors");
		}
		return false;
	}


	// if in sensor test mode do not check for obstacle or abyss and continue measuring
	if (sensorToTest > -1) {
		if (verbose) Serial.println("in sensor test mode, no checking of obstacle or abyss with floor sensors");
		return true;
	}

	// test for OBSTACLE in floor range with infrared sensors
	if (floorObstacleMax > FLOOR_MAX_OBSTACLE) {
		logObstacle(INFRARED, floorObstacleMax, FLOOR_MAX_OBSTACLE, floorObstacleMaxId);
		return false;
	}

	// check ultrasonic distances
	int obstacleSensorId = checkUltrasonicDistances();
	int minObstacleDistance = ultrasonicDistanceSensorValues[obstacleSensorId];

	if (minObstacleDistance < 30) {
		logObstacle(ULTRASONIC, minObstacleDistance, 30, obstacleSensorId);
		return false;
	}

	// test for abyss value in floor distance sensors
	if (floorAbyssMax > FLOOR_MAX_ABYSS) {
		logAbyss(floorAbyssMax, FLOOR_MAX_ABYSS, floorAbyssMaxId);
		return false;
	} 


	// test for docking switch
	if (activeCartMovement == FORWARD 
		&& digitalRead(DOCKING_SWITCH_PIN)
		&& inFinalDockingMove == false) {

		// reset distance to drive into dock to configured value
		inFinalDockingMove = true;
		requestedDistance = totalDistanceMoved + finalDockingMoveDistance;

		Serial.print("docking switch activated, continue FORWARD for ");
		Serial.print(finalDockingMoveDistance);
		Serial.println();
		return true;
	}
		
	// check for free move after detecting obstacle or abyss
	if ((prevMoveStatus == OBSTACLE) || (prevMoveStatus == ABYSS)) {

		// inform cartControl
		Serial.print("!A4");
		Serial.println();

		if (verbose) {
			Serial.println("continue move after stop");
		}
	}
	return true;	// move allowed
}


void stopCart(bool moveDone, String reason) {

	if (verbose) {
		Serial.print("stopCart, moveDone: ");
		Serial.print(moveDone);
		Serial.print(" reason: ");
		Serial.println(reason);
	}
	activeCartMovement = STOP;

	if (moveDone) {

		// move done, stop swiping and log distance
		stopSwipe();
		maxMoveDuration = 0;
		sensorToTest = -1;
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
			Serial.println((String)"!A5,"
				+ int(platformImu.getYaw())
				+ "," + reason);
		}
		else {
			Serial.println((String)"!A6,"
				+ int(platformImu.getYaw())
				+ "," + totalDistanceMoved
				+ "," + reason);
		}

		plannedCartMovement = STOP;
	}
	stopMotors();
}


void handleRotation() {

	if (verbose) {
		Serial.println("handle rotation");
	}

	int angleMoved = absAngleDiff(platformImu.getYaw(), imuYawStart);
	if (verbose) {
		Serial.print("angle moved: ");
		Serial.print(angleMoved);
		Serial.println();
	}

	if (verbose && angleMoved > 0) {
		Serial.print("angleMoved: ");
		Serial.print(angleMoved);
		Serial.print(", imuYaw: ");
		Serial.print(platformImu.getYaw());
		Serial.print(", _startYaw: ");
		Serial.print(imuYawStart);
		Serial.println();

		Serial.print("check rotation done, angleMoved: ");
		Serial.print(angleMoved);
		Serial.print(", requestedAngle: ");
		Serial.print(requestedAngle);
		Serial.println();
	}

	if (abs(requestedAngle) - angleMoved < 1) {
		stopCart(true, "rotation done");

		Serial.println((String)" stop rotation, imuYawStart: " + imuYawStart
			+ ", requested angle: " + requestedAngle
			+ ", angle moved: " + angleMoved
			+ ", imuYawEnd: " + platformImu.getYaw());

		return;
	}


	/////////////////////////////////////////////////////////
	// check for obstacles/abyss in planned cart move
	/////////////////////////////////////////////////////////
	if (freeMoveAvailable(plannedCartMovement)) {

		if (verbose) {
			Serial.print(millis() - msMoveCmd);
			Serial.println(", free rotation available");
		}

		// check for cart currently stopped
		if (cartSpeed == 0) {

			// start with minimal speed and speedPhase ACCELERATE
			cartSpeed = MINIMAL_CART_SPEED;
			speedPhase = ACCELERATE;
			if (verbose) {
				Serial.print("cartSpeed: ");
				Serial.print(cartSpeed);
				Serial.println(", speedPhase ACCELERATE set");
			}
			activeCartMovement = plannedCartMovement;
			moveInterruptedMillis = 0;
		}

		// check for acceleration/deceleration phase
		if (speedPhase == ACCELERATE) {

			// increase speed
			if (cartSpeed < requestedMaxSpeed) {
				cartSpeed += MAX_SPEED_INCREASE;
				accelerationAngle = angleMoved - angleAtInterrupt;
				accelerationTime = moveDuration - durationAtInterrupt;

				// check for end of acceleration phase
				if (cartSpeed >= requestedMaxSpeed) {
					speedPhase = CRUISE;
					if (verbose) Serial.println("speedPhase CRUISE set");

					if (verbose) {
						Serial.print("acceleration done, cartSpeed: ");
						Serial.print(cartSpeed);
						Serial.print(", acceleration angle: ");
						Serial.print(accelerationAngle);
						Serial.print(", acceleration time: ");
						Serial.print(accelerationTime);
						Serial.println();
					}
				}
			}
		}

		// check for deceleration
		int remainingAngle = absAngleDiff(requestedAngle, angleMoved);
		int remainingTime = maxMoveDuration - moveDuration;

		if ((remainingAngle <= (1.0 * accelerationAngle))
			|| (remainingTime <= (1.0 * accelerationTime))) {
			if (speedPhase != DECELERATE) {
				speedPhase = DECELERATE;
				if (verbose) Serial.println("speedPhase DECELERATE set");
			}
		}
		if (speedPhase == DECELERATE) {

			cartSpeed -= MAX_SPEED_DECREASE;
			if (cartSpeed < MINIMAL_CART_SPEED) {
				cartSpeed = MINIMAL_CART_SPEED;
			}
			if (verbose) {
				Serial.print("decelerate remaining angle: ");
				Serial.print(remainingAngle);
				Serial.print(", cartSpeed: ");
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
				Serial.print("cart position update, distance: ");
				Serial.print(totalDistanceMoved);
				Serial.print(", startYaw: ");
				Serial.print(imuYawStart);
				Serial.print(", imuYaw: ");
				Serial.print(platformImu.getYaw());
				Serial.print(", speedLeft: ");
				Serial.print(driftCompensationLeft);
				Serial.print(", speedRight: ");
				Serial.print(driftCompensationRight);
				Serial.println();
			}
		}
	}
}


void handleMove() {

	if (verbose) {
		Serial.print(millis() - msMoveCmd);
		Serial.print(", handle Move, protected: ");
		Serial.print(moveProtected ? "true" : "false");
		Serial.println();
	}

	updateDistanceMoved();

	// check for total distance done
	if (abs(totalDistanceMoved - requestedDistance) < 5) {
		stopCart(true, "requested distance moved");

		Serial.print("stop move, imuYawStart: ");
		Serial.print(imuYawStart);
		Serial.print(", requested distance: ");
		Serial.print(requestedDistance);
		Serial.print(", distance moved: ");
		Serial.print(totalDistanceMoved);
		Serial.print(", imuYawEnd: ");
		Serial.print(platformImu.getYaw());
		Serial.println();

		return;
	}

	if (freeMoveAvailable(plannedCartMovement) || !moveProtected) {

		if (verbose) {
			Serial.print(millis() - msMoveCmd);
			Serial.println(", free move available");
		}

		// check for cart currently stopped
		if ((cartSpeed == 0) && (cartSpeed < requestedMaxSpeed)) {

			// start with minimal speed and speedPhase ACCELERATE
			cartSpeed = MINIMAL_CART_SPEED;
			speedPhase = ACCELERATE;
			if (verbose) Serial.println("speedPhase ACCELERATE set");
		}

		activeCartMovement = plannedCartMovement;
		moveInterruptedMillis = 0;

		// check for speed phase accelerate
		if (speedPhase == ACCELERATE) {

			// increase speed
			if (cartSpeed < requestedMaxSpeed) {
				cartSpeed += MAX_SPEED_INCREASE;
				accelerationDistance = totalDistanceMoved - distanceAtInterrupt;
				accelerationTime = moveDuration - durationAtInterrupt;

				// check for end of acceleration phase
				if (cartSpeed >= requestedMaxSpeed) {
					speedPhase = CRUISE;
					if (verbose) Serial.println("speedPhase CRUISE set");

					if (verbose) {
						Serial.print("acceleration done, cartSpeed: ");
						Serial.print(cartSpeed);
						Serial.print(", acceleration distance: ");
						Serial.print(accelerationDistance);
						Serial.print(", acceleration time: ");
						Serial.print(accelerationTime);
						Serial.println();
					}
				}
			}
		}

		// check for deceleration based on distance moved (may be during acceleration)
		int remainingDistance = requestedDistance - totalDistanceMoved;
		int remainingTime = maxMoveDuration - moveDuration;

		if ((remainingDistance <= (1.0 * accelerationDistance))
			|| (remainingTime <= (1.0 * accelerationTime))) {
			if (speedPhase != DECELERATE) {
				speedPhase = DECELERATE;
				if (verbose) Serial.println("speedPhase DECELERATE set");
			}
		}

		if (speedPhase == DECELERATE) {
			cartSpeed -= MAX_SPEED_DECREASE;
			if (cartSpeed < MINIMAL_CART_SPEED) {
				cartSpeed = MINIMAL_CART_SPEED;
			}
			if (verbose) {
				Serial.print("decelerate remaining distance: ");
				Serial.print(remainingDistance);
				Serial.print(", cartSpeed: ");
				Serial.print(cartSpeed);
				Serial.println();
			}
		}
	
		handleDrift();

		activeCartMovement = plannedCartMovement;
	
		applyCartSpeed();
	}
}


bool moveDurationExpired() {

	// check total move time
	if (moveDuration > maxMoveDuration) {
		stopCart(true, "max move duration reached");

		Serial.print("maxMoveDuration reached, stopping cart after: ");
		Serial.print(moveDuration);
		Serial.print(" ms, maxMoveDuration: ");
		Serial.print(maxMoveDuration);
		Serial.print(" ms, distance: ");
		Serial.print(totalDistanceMoved);
		Serial.print(", imuYaw: ");
		Serial.println(platformImu.getYaw());

		plannedCartMovement = STOP;
		return true;
	}

	// check continuation time after obstacle or abyss detection
	if (moveInterruptedMillis > 0) {
		if ((plannedCartMovement != STOP) 
			&& (millis() - moveInterruptedMillis > MAX_WAIT_FOR_FREE_MOVE)) {

			stopCart(true, "timeout in continuation of move");

			Serial.print("max wait after blocking reached, distance: ");
			Serial.print(totalDistanceMoved);
			Serial.print(", imuYaw: ");
			Serial.println(platformImu.getYaw());

			plannedCartMovement = STOP;
			return true;
		}
	}
	return false;
}


///////////////////////////////////////////////
// main move function called from arduino loop
///////////////////////////////////////////////
void handleCartMovement() {

	if (plannedCartMovement != STOP) {

		if (moveDurationExpired()) {
			Serial.println("moveDurationExpired");
			return;
		}

		// keep track of total move time
		if (activeCartMovement != STOP) {
			moveDuration += (millis() - moveCycleStartMillis);
			if (verbose) {
				Serial.print("move duration: ");
				Serial.println(moveDuration);
			}
			// verify we get encoder signals
			if ((sensorToTest > -1) && (moveDuration > 500) && (wheelPulseCounter == 0)) {
				stopCart(false, "missing encoder signals");
				return;
			}
		}
		moveCycleStartMillis = millis();

		//Serial.println((String)"I102 currentMeasureStep: " + currentMeasureStep
		//	+ ", swipeDirection: " + swipeDirection
		//	+ ", nextMeasureStep: " + nextMeasureStep);
		currentMeasureStep = nextMeasureStep;
		if (nextMeasureStep == NUM_MEASURE_STEPS) {
			swipeDirection = -1;
			currentMeasureStep = NUM_MEASURE_STEPS-1;
			//nextMeasureStep = NUM_MEASURE_STEPS - 2;
		}
		if (nextMeasureStep == -1) {
			swipeDirection = 1;
			currentMeasureStep = 0;
			//nextMeasureStep = 1;
		}
		//Serial.println((String)"I103 currentMeasureStep: " + currentMeasureStep
		//	+ ", swipeDirection: " + swipeDirection
		//	+ ", nextMeasureStep: " + nextMeasureStep);

		if (moveProtected) {

			// try to get a set of reasonable distance sensor values
			// initialize range of measured values to high value
			for (int sensorId = 0; sensorId < FLOOR_SENSORS_COUNT; sensorId++) sensorDataSwipeStep[sensorId].range = DISTANCE_UNKNOWN;
			int numTries = 0;

			// if raw values have high deviation retry the measuring
			while (readDistanceSensorRawValues(plannedCartMovement) > 50 && numTries < 3) {
				numTries += 1;
				Serial.println((String)"I101: high deviation in distance raw values, retry");
			}
			

			// move immediately to next swipe position after reading distance sensor values
			nextSwipeServoStep();

			// process the raw analog distance values measured
			evalObstacleOrAbyss(plannedCartMovement);
		}

		// Get current orientation from imu
		headImu.readBnoSensorData();
		platformImu.readBnoSensorData();

		if (plannedCartMovement == ROTATE_CLOCKWISE || plannedCartMovement == ROTATE_COUNTERCLOCK) {
			handleRotation();
		} else {
			handleMove();
		}


		// send position update to cartControl
		if ((millis() - lastPositionSentMillis > 200) && (activeCartMovement != STOP)) {
		
			lastPositionSentMillis = millis();
			Serial.print("!Aa,");
			Serial.print(int(platformImu.getYaw()));
			Serial.print(",");
			Serial.print(totalDistanceMoved);
			Serial.print(",");
			Serial.print(activeCartMovement);
		
			Serial.println();
		}
	}
}
