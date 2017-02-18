#include "CommandBase.h"

class CenterGear : public CommandBase {
private:
	const double DISTANCE_METERS = 5.0;
public:
	CenterGear();
	virtual ~CenterGear(){}

	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
};
