#include "SetGearServos.h"

SetGearServos::SetGearServos(bool open) : do_open(open) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
}

MoveClawsInCommand::MoveClawsInCommand()   : SetGearServos(false){};
MoveClawsOutCommand::MoveClawsOutCommand() : SetGearServos(true){};

// Called just before this Command runs the first time
void SetGearServos::Initialize() {
	if (do_open)
		MoveClawsOut();
	else
		MoveClawsIn();
}

// Called repeatedly when this Command is scheduled to run
void SetGearServos::Execute() {

}

// Make this return true when this Command no longer needs to run execute()
bool SetGearServos::IsFinished() {
	return true;
}

// Called once after isFinished returns true
void SetGearServos::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void SetGearServos::Interrupted() {

}
