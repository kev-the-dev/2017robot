#include "CommandBase.h"
#include "Timer.h"

class DriveForwardAuto : public CommandBase {
private:
public:
	DriveForwardAuto();
	virtual ~DriveForwardAuto(){}

	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
	Timer timer;
	const double TIME = 3;
	const double EFFORT = 0.5;
};
