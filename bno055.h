// bno055.h

#ifndef _bno055_h
#define _bno055_h

#include "arduino.h"
#include <Adafruit_BNO055.h>

class cImu {

public:
	bool setAddressName(byte thisAddress, String thisName, String thisId);
	//void readBnoSensorData(bool publishOnChangeOnly = true);
	bool readBnoSensorData();
	float getYaw();
	float getRoll();
	float getPitch();
	String getId();
	String getName();
	unsigned long getMillisLastPublished();
	void setMillisLastPublished();

private:
	byte address;
	String name;
	String id;
	Adafruit_BNO055 device;
	float yaw;
	float roll;
	float pitch;
	float prevYaw;
	float prevRoll;
	float prevPitch;
	unsigned long millisLastPublished;
};

#endif