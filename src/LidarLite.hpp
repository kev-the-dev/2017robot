#pragma once

#include <I2C.h>

class LidarLite
{
private:
	enum VALUES {
		RESET = 0x00,
		MEASURE_UNCORRECTED = 0x03,
		MEASURE_CORRECTED = 0x04
	};
	enum REGISTERS {
		STATUS = 0x01,
		ACQ_COMMAND = 0x00,
		DISTANCE = 0x08,
		SIG_COUNT_VAL = 0x02,
		ACQ_CONFIG_REG = 0x04,
		THRESHOLD_BYPASS = 0x1c
	};
	const uint8_t STATUS_REGISTER = 0x01;
	const unsigned int TIMEOUT_MILISECONDS = 200;
	bool WaitForAvailable();
	bool Busy();
	frc::I2C device;
public:
	LidarLite(frc::I2C::Port port=frc::I2C::Port::kOnboard, int device=0x62);
	bool SetAddress(uint8_t serialnumber, int address);
	bool Reset();
	bool configure(int config=0);
	int GetDistance(bool corrected=true);

};
