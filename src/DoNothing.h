#include <Commands/Command.h>

class DoNothing : public Command{
public:
	DoNothing() {};
	virtual ~DoNothing(){}
	bool IsFinished(){return true;}
};

