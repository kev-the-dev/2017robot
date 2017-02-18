#include "CommandBase.h"

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
};
