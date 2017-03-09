#include <TwoMotorOutput.hpp>
TwoMotorOutput::TwoMotorOutput(std::shared_ptr<frc::SpeedController> one, std::shared_ptr<frc::SpeedController> two):
		m_one(one),
		m_two(two),
		last(0.0),
		inverted(false)
{

}
TwoMotorOutput::~TwoMotorOutput(){};
void TwoMotorOutput::Set(double output)
{
	m_one->Set(output);
	m_two->Set(output);
	last = output;
}
void TwoMotorOutput::PIDWrite(double output) {Set(output);}
double TwoMotorOutput::Get() const
{
	return last;
}
void TwoMotorOutput::SetInverted(bool inverted)
{
	this->inverted = inverted;
	m_one->SetInverted(inverted);
	m_two->SetInverted(inverted);
}
void TwoMotorOutput::Disable()
{
	m_one->Disable();
	m_two->Disable();
}
bool TwoMotorOutput::GetInverted() const
{
	return inverted;
}
void TwoMotorOutput::StopMotor()
{
	m_one->StopMotor();
	m_two->StopMotor();
}
