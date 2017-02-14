#include <LidarLite.hpp>

bool LidarLite::WaitForAvailable()
{
	auto timeout = std::chrono::steady_clock::now() + std::chrono::milliseconds(TIMEOUT_MILISECONDS);
	while (std::chrono::steady_clock::now() < timeout)
	{
		if (!Busy()) return true;
	}
	printf("Wait for not busy timed out line %d\n", __LINE__);
	return false;
}
bool LidarLite::Busy()
{
	uint8_t statusWrite = REGISTERS::STATUS;
	uint8_t statusRead = 0;

	if(device.WriteBulk(&statusWrite, 1)) printf("Write failed at %d", __LINE__);
	if(device.ReadOnly(1, &statusRead)) printf("Read failed at %d", __LINE__);
	printf("Status at line %d %0x, bit0=%0x\n", __LINE__, statusRead, statusRead & (unsigned char)0x01);
	return statusRead & ((unsigned char) 0x01);
}
LidarLite::LidarLite(frc::I2C::Port port, int device) :
		device(port, device)
{

}
/*------------------------------------------------------------------------------
  Configure

  Selects one of several preset configurations.

  Parameters
  ------------------------------------------------------------------------------
  configuration:  Default 0.
    0: Default mode, balanced performance.
    1: Short range, high speed. Uses 0x1d maximum acquisition count.
    2: Default range, higher speed short range. Turns on quick termination
        detection for faster measurements at short range (with decreased
        accuracy)
    3: Maximum range. Uses 0xff maximum acquisition count.
    4: High sensitivity detection. Overrides default valid measurement detection
        algorithm, and uses a threshold value for high sensitivity and noise.
    5: Low sensitivity detection. Overrides default valid measurement detection
        algorithm, and uses a threshold value for low sensitivity and noise.
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
bool LidarLite::configure(int configuration)
{
	  switch (configuration)
	  {
	    case 0: // Default mode, balanced performance
	      device.Write(0x02,0x80); // Default
	      device.Write(0x04,0x08); // Default
	      device.Write(0x1c,0x00); // Default
	    break;

	    case 1: // Short range, high speed
	      device.Write(0x02,0x1d);
	      device.Write(0x04,0x08); // Default
	      device.Write(0x1c,0x00); // Default
	    break;

	    case 2: // Default range, higher speed short range
	      device.Write(0x02,0x80); // Default
	      device.Write(0x04,0x00);
	      device.Write(0x1c,0x00); // Default
	    break;

	    case 3: // Maximum range
	      device.Write(0x02,0xff);
	      device.Write(0x04,0x08); // Default
	      device.Write(0x1c,0x00); // Default
	    break;

	    case 4: // High sensitivity detection, high erroneous measurements
	      device.Write(0x02,0x80); // Default
	      device.Write(0x04,0x08); // Default
	      device.Write(0x1c,0x80);
	    break;

	    case 5: // Low sensitivity detection, low erroneous measurements
	      device.Write(0x02,0x80); // Default
	      device.Write(0x04,0x08); // Default
	      device.Write(0x1c,0xb0);
	    break;
	  }
	  return false;
}
bool LidarLite::SetAddress(uint8_t serialnumber, int address)
{
	uint8_t res[2];
	if (device.Read(0x16, 1, &res[0])) return true; // Get Serial high byte
	if (device.Read(0x17, 1, &res[1])) return true; // Get serial low byte
	if (device.Write(0x18, res[0])) return true;   // Write first byte of serial number
	if (device.Write(0x19, res[1])) return true;   // Write second bye of serial number
	if (device.Write(0x1a, address)) return true;
	if (device.Write(0x1e, 0x08)) return true;
	return false;
}
bool LidarLite::Reset()
{
	return device.Write(REGISTERS::ACQ_COMMAND, VALUES::RESET);
}
int LidarLite::GetDistance(bool corrected)
{
	if(!WaitForAvailable()) return -1;
	if (corrected)
	{
		uint8_t commandWrite[2];
		commandWrite[0] = REGISTERS::ACQ_COMMAND;
		commandWrite[1] = VALUES::MEASURE_CORRECTED;
		//if (device.Write(0x00, 0x04)) return -1;
		if (device.WriteBulk(commandWrite, 2))
		{
			printf("Write failed at line%d\n", __LINE__);
			return -1;
		}
	}
	else
	{
		if (device.Write(REGISTERS::ACQ_COMMAND, VALUES::MEASURE_UNCORRECTED))
		{
			printf("Line %d just failed\n", __LINE__);
			return -1;
		}
	}
	if(!WaitForAvailable()) return -1;
	uint8_t readRegister = REGISTERS::DISTANCE;
	if(device.WriteBulk(&readRegister, 1)) printf("Write failed at %d\n", __LINE__);
	uint8_t distanceRead[2];
	if(device.ReadOnly(2, distanceRead)) printf("Read failed at line %d\n", __LINE__);
	int distance = (unsigned int)(distanceRead[0]<<8) + (unsigned int)(distanceRead[1]);
	return distance;
}


