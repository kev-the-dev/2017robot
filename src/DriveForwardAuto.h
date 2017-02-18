#include <Commands/Command.h>

class DriveForwardAuto : public frc::Command {
public:
	DriveForwardAuto();
	virtual ~DriveForwardAuto(){}

	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
};
