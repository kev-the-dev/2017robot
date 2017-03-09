#include "DriveForwardAuto.h"

DriveForwardAuto::DriveForwardAuto()
{

}
void DriveForwardAuto::Initialize()
{
	timer.Reset();
	timer.Start();
	SetWrenchEffort();
}
void DriveForwardAuto::Execute()
{
	drive->ArcadeDrive(0, -EFFORT);
}
bool DriveForwardAuto::IsFinished()
{
	return timer.HasPeriodPassed(TIME);
}
void DriveForwardAuto::End()
{
	drive->Drive(0,0);
	timer.Stop();
}
void DriveForwardAuto::Interrupted()
{
	drive->Drive(0,0);
}

