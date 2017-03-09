#include "EncoderAuto.h"

EncoderAuto::EncoderAuto(int option)
{
	selection = option;
	stage = 0;
}
void EncoderAuto::Initialize()
{
	stage = 0;
	SetWrenchVelocity();
	timer.Reset();
}
bool EncoderAuto::drive(const double position){
	double error = (position - odom->Get().pose.pose.position.x)/fabs(position);
	double goal = KP_METERS_PER_SECOND*error;
	if (fabs(goal) <= MINIMUM_VELOCITY)
	{
		if (goal > 0) goal = MINIMUM_VELOCITY;
		else goal = -MINIMUM_VELOCITY;
	}
	drive_ctrl->Set(goal, 0);
	printf("Position=%f Error = %f Goal = %f\n", position, error, goal);
	if (fabs(error) < 0.01){
		drive_ctrl->Set(0,0);
		return true;
	}
	else return false;
}
bool EncoderAuto::rotate(double degrees){
	double error = (degrees - odom->GetThetaDeg())/fabs(degrees);
	double goal = KP_ANGULAR_RADIANS_SEC*error;
	if (fabs(goal) <= MINIMUM_ANGULAR_VELOCITY)
	{
		if (goal > 0 ) goal = MINIMUM_ANGULAR_VELOCITY;
		else goal = -MINIMUM_ANGULAR_VELOCITY;
	}
	printf("Angular Degrees=%f Error = %f Goal = %f\n", degrees, error, goal);
	drive_ctrl->Set(0, goal);
	if (fabs(error) < 0.01){
		drive_ctrl->Set(0,0);
		return true;
	}
	else return false;
}
void EncoderAuto::Execute()
{
	if (stage == 0){ //Drive forward to gear lift
		if (drive(POSITION_X_1)){
			MoveClawsOut();
			timer.Start();
			stage=1;
		}
	}
	else if (stage == 1){ //Pause
		if (timer.Get() >= TIME_PAUSE_SECONDS) stage = 2;
	}
	else if (stage == 2){ //Drive backward
		timer.Stop();
		if (drive(POSITION_X_2)){
			MoveClawsIn();
			stage = 3;
		}
	}
	else if (stage == 3){ //Rotate
		if (rotate(-selection*ANGLE_TO_ROTATE_DEGREES))
		{
			odom->Reset();
			stage = 4;
		}
	}
	else if (stage == 4){ //Drive left/right
		if (drive(POSITION_X_3)) stage = 5;
	}
	else if (stage == 5){ //Rotate again
		if (rotate(selection*ANGLE_TO_ROTATE_DEGREES)){
			odom->Reset();
			stage = 6;
		}
	}
	else if (stage == 6){//Drive past baseline
		if (drive(POSITION_X_4)) stage = 7;
	}
}
bool EncoderAuto::IsFinished()
{
	return (stage == 7 || (stage == 3 && selection == 0));
}
void EncoderAuto::End()
{
	drive_ctrl->Disable();
}
void EncoderAuto::Interrupted()
{

}
