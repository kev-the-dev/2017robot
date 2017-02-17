#include <TwoMotorOutput.hpp>
TwoMotorOutput::TwoMotorOutput(std::shared_ptr<frc::SpeedController> one, std::shared_ptr<frc::SpeedController> two):
		m_one(one),
		m_two(two)
{

}
TwoMotorOutput::~TwoMotorOutput(){};
void TwoMotorOutput::Set(double output)
{
	m_one->Set(output);
	m_two->Set(output);
}
void TwoMotorOutput::PIDWrite(double output)
{
	Set(output);
}
