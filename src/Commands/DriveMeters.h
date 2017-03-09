#ifndef DriveMeters_H
#define DriveMeters_H

#include "CommandBase.h"

class DriveMeters : public CommandBase {
private:
	static constexpr double MINIMUM_VELOCITY_METERS_PER_SECOND = 0.5;
	static constexpr double MAX_VELOCITY_METERS_PER_SECOND     = 1.5;
	static constexpr double MIN_ERROR                          = 0.01;
	double goal_meters;
	double error;
public:
	DriveMeters(double goal);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DriveMeters_H
