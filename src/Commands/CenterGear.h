#include "CommandBase.h"

class CenterGear : public CommandBase {
private:
	//const double DISTANCE_METERS = 1.8796;
	//const double KP_IN_METERS_PER_SECOND = 1.5;
	//const double MINIMUM_VELOCITY = 0.5;
	const double EFFORT = 0.5;
	const double EFFORT_BACK = 0.7;
	const double TIME_FORWARD = 4;
	const double TIME_PAUSE = TIME_FORWARD+1;
	const double TIME_ROTATE = TIME_PAUSE+0.5;
	const double TIME_FORWARD_2 = TIME_ROTATE+1;
	const double TIME_END = TIME_FORWARD_2+3;
	int selection;
public:
	CenterGear(int option);
	virtual ~CenterGear(){}

	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
	Timer timer;
};
