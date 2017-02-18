#include "CenterGear.h"

CenterGear::CenterGear()
{

}
void CenterGear::Initialize()
{
	drive_ctrl->Enable();
	drive_ctrl->Set(1.5, 0);
}
void CenterGear::Execute()
{

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

