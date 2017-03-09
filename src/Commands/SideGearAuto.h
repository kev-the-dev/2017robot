#ifndef SideGearAuto_H
#define SideGearAuto_H

#include <Commands/CommandGroup.h>
#include <Commands/WaitCommand.h>

#include "DriveMeters.h"
#include "RotateDegrees.h"
#include "SetGearServos.h"

class SideGearAuto : public CommandGroup {
private:
	static constexpr double FORWARD_METERS   = 1.81356;
	static constexpr double ROTATE_DEGREES   = 60.0;
	static constexpr double FORWARD_METERS_2 = 1.778;
	static constexpr double PAUSE_SECONDS    = 3.0;
	static constexpr double BACKUP_METERS    = -1.0;
	bool right;
public:
	// false = left, true = right
	SideGearAuto(bool right);
};

#endif  // SideGearAuto_H
