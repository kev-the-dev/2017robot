#include "CenterGear.h"

CenterGear::CenterGear(int option)
{
selection = option;
}
void CenterGear::Initialize()
{
	timer.Reset();
	timer.Start();
	drive->SetSafetyEnabled(false);
	//drive_ctrl->Enable();
}
void CenterGear::Execute()
{
	/*if (timer.HasPeriodPassed(TIME_PAUSE)) drive->ArcadeDrive(0, EFFORT);
	else if (timer.HasPeriodPassed(TIME_FORWARD)){
		drive->ArcadeDrive(0.0,0);
		MoveClawsOut();
	}
	else drive->ArcadeDrive(0, -EFFORT);
	*/
	double time = timer.Get();
	if (time < TIME_FORWARD) drive->ArcadeDrive(0, -EFFORT);
	if (time > TIME_FORWARD && time < TIME_PAUSE){
		drive->ArcadeDrive(0.0,0);
		MoveClawsOut();
	}
	if (time > TIME_PAUSE && time < TIME_ROTATE) drive->ArcadeDrive(0, EFFORT_BACK);
	if (time > TIME_ROTATE && time < TIME_FORWARD_2) drive->ArcadeDrive(selection*0.7, 0);
	if (time > TIME_FORWARD_2 && time < TIME_END) drive->ArcadeDrive(0, -EFFORT);

	//double error = (DISTANCE_METERS-odom->Get().pose.pose.position.x)/DISTANCE_METERS;
	//if (KP_IN_METERS_PER_SECOND*error <= MINIMUM_VELOCITY) drive_ctrl->Set(MINIMUM_VELOCITY, 0);
	//else drive_ctrl->Set(KP_IN_METERS_PER_SECOND*error, 0);
}
bool CenterGear::IsFinished()
{
	//return odom->Get().pose.pose.position.x >= DISTANCE_METERS;
	return (timer.HasPeriodPassed(TIME_END) || (timer.Get() > TIME_ROTATE && selection == 0));
}
void CenterGear::End()
{
	drive->ArcadeDrive(0.0,0);
	MoveClawsIn();
	timer.Stop();
	//drive_ctrl->Disable();
}
void CenterGear::Interrupted()
{
	//drive_ctrl->Disable();
	drive->ArcadeDrive(0.0,0);
}

