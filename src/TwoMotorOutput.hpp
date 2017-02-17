#pragma once

#include <PIDController.h>
#include <SpeedController.h>

class TwoMotorOutput: public frc::PIDOutput
{
private:
	std::shared_ptr<frc::SpeedController> m_one;
	std::shared_ptr<frc::SpeedController> m_two;
public:
	TwoMotorOutput(std::shared_ptr<frc::SpeedController> one, std::shared_ptr<frc::SpeedController> two);
	virtual ~TwoMotorOutput();
	void PIDWrite(double output) override;
	void Set(double output);
};
