#include "DriveForwardAuto.h"

DriveForwardAuto::DriveForwardAuto()
{

}
void DriveForwardAuto::Initialize()
{
	timer.Reset();
	timer.Start();
}
void DriveForwardAuto::Execute()
{
	drive->Drive(EFFORT,0);

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

