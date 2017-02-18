#pragma once

#include "RobotMap.h"
#include <Commands/Command.h>

class CommandBase : public frc::Command, public RobotMap {
public:
	CommandBase(){};
	virtual ~CommandBase(){};
};
