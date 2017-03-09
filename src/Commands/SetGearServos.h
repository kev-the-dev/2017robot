#ifndef SetGearServos_H
#define SetGearServos_H

#include "CommandBase.h"

class SetGearServos : public CommandBase {
private:
	bool do_open;
public:
	SetGearServos(bool open);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

class MoveClawsInCommand: public SetGearServos
{
public:
	MoveClawsInCommand();
};
class MoveClawsOutCommand: public SetGearServos
{
public:
	MoveClawsOutCommand();
};


#endif  // SetGearServos_H
