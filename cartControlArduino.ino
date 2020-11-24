// 
// 
// 


#include <Adafruit_BNO055.h>
#include <math.h>
#include <Wire.h>
#include <EEPROM.h>

#include "cartControlArduino.h"
#include "drive.h"
#include "distance.h"
#include "table.h"
#include "bno055.h"
#include "communication.h"


bool verbose = false;

///////////////////////////////////////////////////////////////////////////////
// these values may get overwritten by cartControl with message 'a'
int FLOOR_MAX_OBSTACLE = 15;
int FLOOR_MAX_ABYSS = 20;
int NUM_REPEATED_MEASURES = 7;
int DELAY_BETWEEN_ANALOG_READS = 20;	// may get overwritten by cart control
int MIN_MEASURE_CYCLE_DURATION = 80;  // may get overwritten by cart control
int finalDockingMoveDistance = 12;		//cm!
////////////////////////////////////////////////////////////////////////////////

char* MOVEMENT_STATUS_NAMES[] = {"MOVING", "OBSTACLE", "ABYSS", "STOPPED"};
char* MOVEMENT_NAMES[] = { "STOP", "FORWARD", "FOR_DIAG_RIGHT", "FOR_DIAG_LEFT",
  "LEFT", "RIGHT", "BACKWARD", "BACK_DIAG_RIGHT", "BACK_DIAG_LEFT",
  "ROTATE_COUNTERCLOCK", "ROTATE_CLOCKWISE" };

////////////////////////////////////////////
// global values
////////////////////////////////////////////
int loopCount;
unsigned long loopStartMs;
unsigned long workMs;
int avgWait = 0;

// global variable for table
int _currentTableHeight;
int _requestedTableHeight;
TABLE_STATUS _tableStatus;

float _speedFactor = 1.527;
float _speedOffset = 52.9;
float _speedFactorSideway = 0.6;
float _speedFactorDiagonal = 0.7;

byte platformBnoAddress = 0x28;
byte headBnoAddress = 0x29;

cImu headImu;
cImu platformImu;

int targetYaw;
int startingYaw = 0;
int currentYaw = 0;

int cartCamPosition = 0;		// pointing down
int cartCamServoPin = 13;

// serial commands
boolean cartControlActive = false;
char receivedChars[64]; // an array to store the received data
int numChars = 0;
bool newData = false;
bool configurationComplete = false;

//int lastCurrent = 0;
int testDirection;

// last check on 12V supply
unsigned long _last12VCheckMillis;
int _mVolts;

// docking state
boolean isDocked = false;

// wheel pulses
volatile unsigned long wheelPulseCounter = 0;

int _speedUnifyer[MOTORS_COUNT]{ 104,100,96,82 };

int sensorToTest = -1;

// we use only 1 interrupt and simply count the flanks as we know the direction
void wheelChange() {
	wheelPulseCounter += 1;         
}


void initializeUltrasonicSensors() {

	Wire.begin();		// master needs no address

	// try to send start measure message to subdevice
	Wire.beginTransmission(ULTRASONIC_DISTANCE_DEVICE_I2C_ADDRESS);
	Wire.write(1);
	Wire.endTransmission();

	// allow subdevice to capture echos
	delay(500);

	// try to get distances
	Serial.println("request ultrasonic distances from Arduino nano over I2C");
	int numBytes = Wire.requestFrom(ULTRASONIC_DISTANCE_DEVICE_I2C_ADDRESS, 1 + ULTRASONIC_DISTANCE_SENSORS_COUNT);

	// first byte is validity mask
	if (Wire.available()) ultrasonicDistanceSensorValidity = Wire.read();
	Serial.print("validity: "); Serial.print(ultrasonicDistanceSensorValidity, BIN); Serial.print(", distances: ");

	int usSensorId = 0;
	while (Wire.available()) {
		ultrasonicDistanceSensorValues[usSensorId] = Wire.read();
		Serial.print(ultrasonicDistanceSensorValues[usSensorId]); Serial.print(", ");
		usSensorId++;
	}
	Serial.println();

	// send stop measure command
	Serial.println("stop measuring ultrasonic distances");
	Wire.beginTransmission(ULTRASONIC_DISTANCE_DEVICE_I2C_ADDRESS);
	Wire.write(2);
	Wire.endTransmission();

	Serial.println("ultrasonic sensors available");
}

//////////////////////////////////////////////////////////////
void setup()
{
	// the power pins
	for (int i = 30; i < 34; i++) {
		digitalWrite(i, HIGH);		// should switch relais off (apply before setting pinmode!)
		pinMode(i, OUTPUT);
	}

	// show Relais Board 1 is working by activating IN4 for 1 second 
	digitalWrite(PIN_RELAIS_TEST, LOW);
	delay(1000);
	digitalWrite(PIN_RELAIS_TEST, HIGH);

	Serial.begin(115200);

	Serial.println("MotorizedBase v1.6");

	// wait for received configuration values
	unsigned long timeoutWait = millis() + 5000;
	while (!configurationComplete && millis() < timeoutWait) { 
		delay(500);
		checkCommand();
	}
	if (millis() > timeoutWait) {
		Serial.println("no configuration values received, using default values");
	} else {
		Serial.println("configuration values received, Arduino continues setup");
	}
	// log the active values
	Serial.print(" FLOOR_MAX_OBSTACLE: "); Serial.println(FLOOR_MAX_OBSTACLE);
	Serial.print(" FLOOR_MAX_ABYSS: "); Serial.println(FLOOR_MAX_ABYSS);
	Serial.print(" NUM_REPEATED_MEASURES: "); Serial.println(NUM_REPEATED_MEASURES);
	Serial.print(" DELAY_BETWEEN_ANALOG_READS: "); Serial.println(DELAY_BETWEEN_ANALOG_READS);
	Serial.print(" MIN_MEASURE_CYCLE_DURATION: "); Serial.println(MIN_MEASURE_CYCLE_DURATION);
	Serial.print(" finalDockingMoveDistance: "); Serial.println(finalDockingMoveDistance);

	setupFahren();

	// if docking switch is active run distance checks with the sensors
	Serial.println("check for activated docking switch");
	delay(2000);
	pinMode(CHARGE_6V_BATTERY_PIN, OUTPUT);
	// set docking switch port to input
	pinMode(DOCKING_SWITCH_PIN, INPUT);
	int dockingSwitch = digitalRead(DOCKING_SWITCH_PIN);

	setupSwipeServos(dockingSwitch);

	tableSetup();

	// the imu's
	while (!platformImu.setAddressName(platformBnoAddress, "platformImu", "!Ah")) {
		Serial.println("could not connect with platform imu");
		delay(100);
	}
	Serial.println("connected with platform imu");
	sendImuValues(platformImu);

	while (!headImu.setAddressName(headBnoAddress, "headImu", "!Ai")) {
		Serial.println("could not connect with head imu");
		delay(100);
	}
	Serial.println("connected with head imu");
	sendImuValues(headImu);

	loadFloorReferenceDistances();

	initializeUltrasonicSensors();

	// set power relais pins to output
	pinMode(CHARGE_LAPTOP_BATTERY_PIN, OUTPUT);
	pinMode(CHARGE_12V_BATTERY_PIN, OUTPUT);

		// set initial docking switch state to undocked
	isDocked = false;

	// and deactivate power relais
	digitalWrite(CHARGE_LAPTOP_BATTERY_PIN, HIGH);
	digitalWrite(CHARGE_6V_BATTERY_PIN, HIGH);
	digitalWrite(CHARGE_12V_BATTERY_PIN, HIGH);

	// make shure to show voltage with first movement after restart
	_last12VCheckMillis = 0;

	// set wheel encoder pins and interrupt routines
	pinMode(WHEEL_ENCODER_PIN_A, INPUT_PULLUP);

	// wheel encoder counts by interrupt
	attachInterrupt(digitalPinToInterrupt(WHEEL_ENCODER_PIN_A), wheelChange, CHANGE);

	Serial.println("Arduino setup done");
	Serial.println("!A0");		//cart ready");
}


void loop()
{
	static int remainingDistance;
	static int remainingTime;
	static int remainingYaw;
	static int movementDuration;
	static int maxDuration;
	static int delayMillis;
	static boolean dockingSwitchState;
	static float distance;
	static unsigned long counts;

	loopStartMs = millis();
	loopCount++;
	
	//Serial.print("loopCount: ");  Serial.println(loopCount);

	handleCartMovement();

	// check for new commands
	checkCommand();


	// check for changed imu values
	// limit updates to 10 per second
	if (headImu.readBnoSensorData()) {
		if (millis() - headImu.getMillisLastPublished() > 100) {
			sendImuValues(headImu);
			headImu.setMillisLastPublished();
		}
	}
	if (platformImu.readBnoSensorData()) {
		if (millis() - platformImu.getMillisLastPublished() > 100) {
			sendImuValues(platformImu);
			int obstacleSensorId = checkUltrasonicDistances();	// update gui
			platformImu.setMillisLastPublished();
		}
	}

	// check for heartbeat from controlling computer
	unsigned long diff = millis() - lastMsg;
	if (cartControlActive && diff > 15000UL) {  // 500, use longer times for manual tests
		Serial.print("timeout heartbeat messages, stop cart, time: "); 
		Serial.print(millis()); Serial.print(" lastMsg: "); Serial.print(lastMsg); 
		Serial.print(" diff: ");  Serial.println(diff);
		cartControlActive = false;
		stopCart(true, "no heartbeat from server");
	}


	// check docking switch change
	dockingSwitchState = digitalRead(DOCKING_SWITCH_PIN);
	if (dockingSwitchState != isDocked) {
		isDocked = dockingSwitchState;

			if (isDocked) {
				Serial.println("!Ad");		// cart is docked

				// aktivate power relais
				digitalWrite(CHARGE_LAPTOP_BATTERY_PIN, LOW);
				digitalWrite(CHARGE_6V_BATTERY_PIN, LOW);
				digitalWrite(CHARGE_12V_BATTERY_PIN, LOW);
			}
			else {
				Serial.println("!Ae");		// cart has undocked

				// deactivate power relais
				digitalWrite(CHARGE_LAPTOP_BATTERY_PIN, HIGH);
				digitalWrite(CHARGE_6V_BATTERY_PIN, HIGH);
				digitalWrite(CHARGE_12V_BATTERY_PIN, HIGH);
			}
	}

	// this compensates runtime differences in loop cycles due to actions required
	// if all time is used up start next loop
	// if partial time is used wait until the next planned cycle start
	workMs = millis() - loopStartMs;

	// when not in move use a fixed cycle time
	// if the cart is not moving we do not need to check for move end and free move space
	if (plannedCartMovement == STOP) {
		delay(20);			//minimal wait
		avgWait += 20;
	}
	else {
		// in order to let the swipe servos arrive at their destinations
		// wait depending on process time used in this loop
		if (workMs < MIN_MEASURE_CYCLE_DURATION) {
			avgWait += MIN_MEASURE_CYCLE_DURATION - workMs;
			delay(MIN_MEASURE_CYCLE_DURATION - workMs);
		}
	}

	// monitor cpu free time
	if (loopCount > 200) {
		//Serial.print("avg wait ms in loop: "); Serial.println(avgWait / loopCount);
		avgWait = 0;
		loopCount = 0;
	}
}