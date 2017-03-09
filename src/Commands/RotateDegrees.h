#ifndef RotateDegrees_H
#define RotateDegrees_H

#include "CommandBase.h"

class RotateDegrees : public CommandBase {
private:
	static constexpr double MINIMUM_ANGULAR_VELOCITY = 2.8;
	static constexpr double MAX_ANGULAR_VELOCITY     = 4.0;
	static constexpr double MIN_ERROR                = 0.01;
	double goal_degrees;
	double error;
public:
	RotateDegrees(double goal);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // RotateDegrees_H
