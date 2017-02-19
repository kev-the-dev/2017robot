#include "CommandBase.h"

class CenterGear : public CommandBase {
private:
	const double DISTANCE_METERS = 1.8796;
	const double KP_IN_METERS_PER_SECOND = 1.5;
	const double MINIMUM_VELOCITY = 0.5;
public:
	CenterGear();
	virtual ~CenterGear(){}

	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
};
