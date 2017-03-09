#include "DriveMeters.h"

DriveMeters::DriveMeters(double goal) : goal_meters(goal), error(0.0) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void DriveMeters::Initialize() {
	odom->Reset();
	SetWrenchVelocity();
}

// Called repeatedly when this Command is scheduled to run
void DriveMeters::Execute() {
	error = (goal_meters - odom->GetX())/fabs(goal_meters);
	double goal = MAX_VELOCITY_METERS_PER_SECOND*error;
	if (fabs(goal) <= MINIMUM_VELOCITY_METERS_PER_SECOND)
	{
		if (goal > 0) goal = MINIMUM_VELOCITY_METERS_PER_SECOND;
		else goal = -MINIMUM_VELOCITY_METERS_PER_SECOND;
	}
	drive_ctrl->Set(goal, 0);
	printf("Position=%fm Error = %f Goal = %f\n", goal_meters, error, goal);
}

// Make this return true when this Command no longer needs to run execute()
bool DriveMeters::IsFinished() {
	if (fabs(error) < MIN_ERROR) {
		return true;
	}
	else return false;
}

// Called once after isFinished returns true
void DriveMeters::End() {
	SetWrenchEffort();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveMeters::Interrupted() {
	End();
}
