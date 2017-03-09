#pragma once

#include <PIDController.h>
#include <SpeedController.h>

class TwoMotorOutput: public frc::SpeedController
{
private:
	std::shared_ptr<frc::SpeedController> m_one;
	std::shared_ptr<frc::SpeedController> m_two;
	double last;
	bool inverted;
public:
	TwoMotorOutput(std::shared_ptr<frc::SpeedController> one, std::shared_ptr<frc::SpeedController> two);
	virtual ~TwoMotorOutput();
	void Set(double output) override;
	void PIDWrite(double output) override;
	double Get() const override;
	void SetInverted(bool inverted) override;
	void Disable() override;
	bool GetInverted() const override;
	void StopMotor() override;
};
