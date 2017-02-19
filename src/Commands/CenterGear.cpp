#include "CenterGear.h"

CenterGear::CenterGear()
{

}
void CenterGear::Initialize()
{
	drive_ctrl->Enable();
}
void CenterGear::Execute()
{
	double error = (DISTANCE_METERS-odom->Get().pose.pose.position.x)/DISTANCE_METERS;
	if (KP_IN_METERS_PER_SECOND*error <= MINIMUM_VELOCITY) drive_ctrl->Set(MINIMUM_VELOCITY, 0);
	else drive_ctrl->Set(KP_IN_METERS_PER_SECOND*error, 0);
}
bool CenterGear::IsFinished()
{
	return odom->Get().pose.pose.position.x >= DISTANCE_METERS;
}
void CenterGear::End()
{
	MoveClawsOut();
	drive_ctrl->Disable();
}
void CenterGear::Interrupted()
{
	drive_ctrl->Disable();
}

