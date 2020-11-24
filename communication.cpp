// 
// 
// 
#include <Arduino.h>
#include <EEPROM.h>

#include "cartControlArduino.h"
#include "communication.h"
#include "drive.h"
#include "bno055.h"
#include "distance.h"


int baseTimeRotation = 2500;	// add this time for rotational max duration limit

// heartbeat
unsigned long lastMsg;

// fill buffer with message
// set newData flag to true, processing in motorizedBase.cpp

void sendImuValues(cImu imu) {

	long milliYaw = imu.getYaw() * 1000;
	long milliRoll = imu.getRoll() * 1000;
	long milliPitch = imu.getPitch() * 1000;

	Serial.println((String)imu.getId()
		+ "," + milliYaw
		+ "," + milliRoll
		+ "," + milliPitch);
}


void recvWithEndMarker() {
	static byte ndx = 0;
	char endMarker = '\n';
	char rc;

	while (Serial.available() > 0 && newData == false) {
		rc = Serial.read();

		if (rc != endMarker) {
			receivedChars[ndx] = rc;
			numChars = ndx;
			ndx++;
			if (ndx >= 60) {
				ndx = 60 - 1;
			}
		}
		else {
			receivedChars[ndx] = '\0'; // terminate the string
			ndx = 0;
			newData = true;
			cartControlActive = true;
			lastMsg = millis();
			//Serial.print("message received, millis: ");
			//Serial.print(millis()); Serial.print(" lastMsg: ");
			//Serial.println(lastMsg);
		}
	}
}


void checkCommand() {

	String msg;
	int dir;
	int speed;
	int newSpeed;
	double ox;
	int servoID;
	int sensorId;
	String sensorDistance;
	int testDirection;
	char msgCopyForParsing[64];
	char * strtokIndx; // this is used by strtok() as an index

	int thisDir;
	int thisRequestedDistance;
	int thisRequestedMaxSpeed;
	int thisMaxMoveDuration;
	int thisRequestedAngle;
	int thisFlagProtected;
	bool thisMoveProtected;

	int eepromStartAddr;
	int distance;

	if (Serial.available() > 0) {
		recvWithEndMarker();
		//standalone: SerialMonitor, select Line Feed and send command, 
		// e.g. 1,1,200,2000 for go forward 2000 mm with speed 200
	}

	if (newData) {

		char mode = receivedChars[0];
		strncpy(msgCopyForParsing, receivedChars, numChars);

		// do not log the heartbeat and the orientation query
		if (mode != '5' && mode != '6' && mode != '9') {
			//if (verbose) {
			if (true) {
				Serial.println((String) "new command, mode: " + mode + ", msg: " + receivedChars);
			}
		}

		
		// if switch is not working check for new variable assignments within switch section and remove them
		switch (mode) {		
			
		case '1':  // move dir 1=forward .. 6=backward, e.g. 1,<dir d1>,<speed d3>,<distance d4>,<max duration d4>

			strtokIndx = strtok(msgCopyForParsing, ","); // msgId, "1"

			strtokIndx = strtok(NULL, ","); // direction
			thisDir = atoi(strtokIndx);

			strtokIndx = strtok(NULL, ","); // speed
			thisRequestedMaxSpeed = atoi(strtokIndx);

			strtokIndx = strtok(NULL, ","); // distance
			thisRequestedDistance = atoi(strtokIndx);

			strtokIndx = strtok(NULL, ","); // duration
			thisMaxMoveDuration = atoi(strtokIndx);

			strtokIndx = strtok(NULL, ","); // flagProtected
			thisMoveProtected = atoi(strtokIndx) == 1;

			setPlannedCartMove(MOVEMENT(thisDir), thisRequestedMaxSpeed, thisRequestedDistance, thisMaxMoveDuration, thisMoveProtected);

			break;

		case '2':  // rotate counterclockwise or left: 2,<angle>,<speed>

			strtokIndx = strtok(msgCopyForParsing, ","); // msgId, "2"

			strtokIndx = strtok(NULL, ","); // angle
			thisRequestedAngle = atoi(strtokIndx);

			strtokIndx = strtok(NULL, ","); // speed
			thisRequestedMaxSpeed = atoi(strtokIndx);

			strtokIndx = strtok(NULL, ","); // protected flag
			thisMoveProtected = atoi(strtokIndx) == 1;

			// set max duration
			thisMaxMoveDuration = baseTimeRotation + thisRequestedAngle * 50;

			setPlannedCartRotation(ROTATE_COUNTERCLOCK, thisRequestedMaxSpeed, thisRequestedAngle, thisMaxMoveDuration, thisMoveProtected);

			break;


		case '3':  // rotate clockwise or right, 3 digits for angle, angle positive

			//msg = receivedChars;
			//relAngle = msg.substring(2, 5).toInt();
			//_requestedMaxSpeed = msg.substring(6, 9).toInt();

			strtokIndx = strtok(msgCopyForParsing, ","); // msgId, "3"

			strtokIndx = strtok(NULL, ","); // angle
			thisRequestedAngle = atoi(strtokIndx);

			strtokIndx = strtok(NULL, ","); // speed
			thisRequestedMaxSpeed = atoi(strtokIndx);

			strtokIndx = strtok(NULL, ","); // protected flag
			thisMoveProtected = atoi(strtokIndx) == 1;

			// set max duration
			thisMaxMoveDuration = baseTimeRotation + thisRequestedAngle * 50;

			setPlannedCartRotation(ROTATE_CLOCKWISE, thisRequestedMaxSpeed, thisRequestedAngle, thisMaxMoveDuration, thisMoveProtected);

			break;


		case '4':  // stop
			stopCart(true, "stop requested by cartControl");
			break;


		case '5':  // getCartOrientation
				   // !A9,<orientation>
			platformImu.readBnoSensorData();
			sendImuValues(platformImu);

			headImu.readBnoSensorData();
			sendImuValues(headImu);

			break;

		case '6':  // set new speed
			msg = receivedChars;
			newSpeed = msg.substring(1, 4).toInt();
			//setCartSpeed(newSpeed);
			Serial.print("set new speed received but ignored: ");
			Serial.print(newSpeed);
			Serial.println();
			break;

		case '7':  // test IR sensor

			MOVEMENT cartDirection;

			strtokIndx = strtok(msgCopyForParsing, ","); // msgId, "7"

			strtokIndx = strtok(NULL, ","); // next item
			sensorToTest = atoi(strtokIndx);    // sensorId

			Serial.println((String)"sensorToTest, Id: " + sensorToTest + ", sensorName: " + floorSensorDefinitions[sensorId].sensorName);

			// set a move direction that includes the sensor to test
			if (sensorToTest >= 0 && sensorToTest < 3) {
				cartDirection = FORWARD;
			}
			if (sensorToTest >= 3 && sensorToTest < 6) {
				cartDirection = BACKWARD;
			}
			if (sensorToTest == 6 || sensorToTest == 7) {
				cartDirection = LEFT;
			}
			if (sensorToTest == 8 || sensorToTest == 9) {
				cartDirection = RIGHT;
			}

			//setPlannedCartMove(MOVEMENT cartAction, int maxSpeed, int dDistance, int duration, bool moveProtected) {
			setPlannedCartMove(cartDirection, 0, 2000, 5000, true);

			break;


		case '8':  // move table 8,<height>

			strtokIndx = strtok(msgCopyForParsing, ","); // msgId, "8"

			strtokIndx = strtok(NULL, ","); // height
			_requestedTableHeight = atoi(strtokIndx);

			_currentTableHeight = getTableHeight();
			Serial.print("request for table height: ");
			Serial.print(_requestedTableHeight);
			Serial.print(", current height: ");
			Serial.print(_currentTableHeight);
			Serial.println();
			delay(200);

			moveTableToHeight(_requestedTableHeight);

			break;


		case '9':  // heart beat
			cartControlActive = true;
			break;

		case 'a':	// config values for max obstacle, abyss, delay, finalDockingMoveDistance

			strtokIndx = strtok(msgCopyForParsing, ","); // msgId, "a"

			strtokIndx = strtok(NULL, ",");  // next item
			FLOOR_MAX_OBSTACLE = atoi(strtokIndx);     // convert this part to an integer

			strtokIndx = strtok(NULL, ",");  // next item
			FLOOR_MAX_ABYSS = atoi(strtokIndx);

			strtokIndx = strtok(NULL, ",");  // next item
			NUM_REPEATED_MEASURES = atoi(strtokIndx);
			if (NUM_REPEATED_MEASURES > MAX_REPEATED_MEASURES) {
				NUM_REPEATED_MEASURES = MAX_REPEATED_MEASURES;
				Serial.print("NUM_REPEATED_MEASURES (");
				Serial.print(NUM_REPEATED_MEASURES);
				Serial.print(") limited to MAX_REPEATED_MEASURES (");
				Serial.print(MAX_REPEATED_MEASURES);
				Serial.println();
			}

			strtokIndx = strtok(NULL, ",");  // next item
			DELAY_BETWEEN_ANALOG_READS = atoi(strtokIndx);

			strtokIndx = strtok(NULL, ",");  // next item
			MIN_MEASURE_CYCLE_DURATION = atoi(strtokIndx);

			strtokIndx = strtok(NULL, ",");  // next item
			finalDockingMoveDistance = atoi(strtokIndx);

			break;

		case 'b':	// speed calculation numbers
			msg = receivedChars;
			_speedFactor = msg.substring(2, 9).toFloat();
			_speedOffset = msg.substring(10, 17).toFloat();
			_speedFactorSideway = msg.substring(18, 23).toFloat();
			_speedFactorDiagonal = msg.substring(24, 29).toFloat();

			Serial.print("msg b: cart speed calculation, factor: "); Serial.println(_speedFactor);
			Serial.print(" offset: "); Serial.println(_speedOffset);
			Serial.print(" factor sideway moves: "); Serial.println(_speedFactorSideway);
			Serial.print(" factor diagonal moves: "); Serial.println(_speedFactorDiagonal);

			break;


		case 'd':	// speed adjustments for all motors in percent in order fr,fl,br,bl
			
			strtokIndx = strtok(msgCopyForParsing, ","); // msgId, "d"
			int speedPercentage;

			for (int i = 0; i < MOTORS_COUNT; i++) {
				strtokIndx = strtok(NULL, ","); // next item
				speedPercentage = atoi(strtokIndx);    // convert this part to an integer
				_speedUnifyer[i] = speedPercentage;
			}

			Serial.print("msg d: motor speed unifyers (fr,fl,br,bl): ");
			for (int i = 0; i < MOTORS_COUNT; i++) {
				Serial.print(_speedUnifyer[i]);
				Serial.print(",");
			}
			Serial.println();

			break;


		case 'e':	// all configuration data received
			// this enables setup to continue
			configurationComplete = true;
			Serial.println("msg e: configuration data complete");

			break;


		case 'f':	// for given sensor set measured distances as floor distances

			strtokIndx = strtok(msgCopyForParsing, ","); // msgId, "f"

			strtokIndx = strtok(NULL, ","); // next item
			sensorId = atoi(strtokIndx);    // sensorId

			Serial.println("set floor reference distances");
			eepromStartAddr = sensorId * NUM_MEASURE_STEPS;

			for (int swipeStep = 0; swipeStep < NUM_MEASURE_STEPS; swipeStep++) {
				strtokIndx = strtok(NULL, ","); // next item
				distance = atoi(strtokIndx);    // sensorId

				EEPROM.write(eepromStartAddr + swipeStep, distance);
				floorSensorReferenceDistances[sensorId][swipeStep] = distance;
				
			}
			break;


		case 'v':	// set verbose state v,<0/1>
			if (receivedChars[2] == '0') {
				verbose = false;
				Serial.println("verbose turned off");
			}
			else {
				verbose = true;
				Serial.println("verbose turned on");
			}
			break;




		default:
			Serial.print("unknown serial command, mode: "); Serial.println(char(mode));
			break;
		}

		newData = false;
	}
}
