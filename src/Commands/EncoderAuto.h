#pragma once
#include "CommandBase.h"
#include "Timer.h"

class EncoderAuto : public CommandBase {
private:
	static constexpr double KP_METERS_PER_SECOND       = 1.5;
	static constexpr double MINIMUM_VELOCITY           = 0.5;
	static constexpr double KP_ANGULAR_RADIANS_SEC    = 4;
	static constexpr double MINIMUM_ANGULAR_VELOCITY  = 2.8;
	static constexpr double POSITION_X_1            = 1.8796;
	static constexpr double TIME_PAUSE_SECONDS         = 0.5;
	static constexpr double POSITION_X_2                 = 1;
	static constexpr double ANGLE_TO_ROTATE_DEGREES     = 90;
	static constexpr double POSITION_X_3                = 1.624;
	static constexpr double POSITION_X_4               = 1.778;

	int selection;
	int stage;

	bool drive(const double position);
	bool rotate(double degrees);

public:
	EncoderAuto(int option);
	virtual ~EncoderAuto(){};
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
	Timer timer;
};

