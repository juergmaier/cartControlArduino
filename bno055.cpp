#include <Adafruit_BNO055.h>
#include "bno055.h"

extern bool verbose;

bool cImu::setAddressAndName(byte thisAddress, String thisName, String thisId) {

	address = thisAddress;
	name = thisName;
	id = thisId;

	Serial.print(F("check i2c address 0x"));
	Serial.print(address, HEX);
	Serial.println();
	device = Adafruit_BNO055(55, address);

	/* Initialise the sensor */
	Serial.print(F("Check for BNO055 "));
	Serial.print(name);
	Serial.println();

	if (!device.begin())
	{
		/* There was a problem detecting the BNO055 ... check your connections */
		Serial.println(F("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!"));
		return false;
	}
	Serial.println(F("BNO055 connected"));
	
	device.setExtCrystalUse(true);

	changedBnoSensorData();
	delay(50);
	changedBnoSensorData();
	return true;
}

float cImu::getYaw() { return yaw;  }
float cImu::getRoll() { return roll; }
float cImu::getPitch() { return pitch; }
String cImu::getId() { return id; }
String cImu::getName() { return name; }
unsigned long cImu::getMillisLastPublished() { return millisLastPublished; }
void cImu::setMillisLastPublished() { millisLastPublished = millis(); }


bool cImu::changedBnoSensorData() {

	sensors_event_t bnoData;
	bool dataChanged = false;

	device.getEvent(&bnoData);

	// normalize, depends on sensor mounting
	yaw = bnoData.orientation.x;
	pitch = bnoData.orientation.y;
	roll = bnoData.orientation.z;	// Correction?

	if (abs(prevYaw - yaw) > 0.5 && abs(prevYaw - yaw) < 359.5) {
		dataChanged = true;
		prevYaw = yaw;
	}

	if (abs(prevRoll - roll) > 1) {
		dataChanged = true;
		prevRoll = roll;
	}

	if (abs(prevPitch - pitch) > 0.5) {
		dataChanged = true;
		prevPitch = pitch;
	}
	return dataChanged;
}



