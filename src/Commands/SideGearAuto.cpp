#include "SideGearAuto.h"

SideGearAuto::SideGearAuto(bool r): right(r)
{
	// Move Forward
	AddSequential(new DriveMeters(FORWARD_METERS));

	// Rotate
	if (right)
		AddSequential(new RotateDegrees(ROTATE_DEGREES));
	else
		AddSequential(new RotateDegrees(-ROTATE_DEGREES));

	// Move Forward Again
	AddSequential(new RotateDegrees(FORWARD_METERS_2));

	// Open Claws
	AddSequential(new MoveClawsOutCommand());

	// Wait a bit
	AddSequential(new WaitCommand(PAUSE_SECONDS));
	// Back Up
	AddSequential(new DriveMeters(BACKUP_METERS));
}
