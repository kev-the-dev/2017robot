#include "RotateDegrees.h"

RotateDegrees::RotateDegrees(double goal) : goal_degrees(goal), error(0.0) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void RotateDegrees::Initialize() {
	odom->Reset();
	SetWrenchVelocity();
}

// Called repeatedly when this Command is scheduled to run
void RotateDegrees::Execute() {
	error = (goal_degrees - odom->GetThetaDeg())/fabs(goal_degrees);
	double goal = MAX_ANGULAR_VELOCITY*error;
	if (fabs(goal) <= MINIMUM_ANGULAR_VELOCITY)
	{
		if (goal > 0) goal = MINIMUM_ANGULAR_VELOCITY;
		else goal = -MINIMUM_ANGULAR_VELOCITY;
	}
	drive_ctrl->Set(goal, 0);
	printf("Position=%fdeg Error = %f Goal = %f\n", goal_degrees, error, goal);
}

// Make this return true when this Command no longer needs to run execute()
bool RotateDegrees::IsFinished() {
	if (fabs(error) < MIN_ERROR) {
		return true;
	}
	else return false;
}

// Called once after isFinished returns true
void RotateDegrees::End() {
	SetWrenchEffort();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void RotateDegrees::Interrupted() {
	End();
}
