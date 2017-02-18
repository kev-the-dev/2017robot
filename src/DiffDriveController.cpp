#include "DiffDriveController.hpp"

DiffDriveController::DiffDriveController(ros::NodeHandle& nh, char* topic, std::shared_ptr<frc::PIDController> left, std::shared_ptr<frc::PIDController> right, double separation):
		m_left(left),
		m_right(right),
		m_separation(separation),
		m_sub(topic, [this] (const geometry_msgs::Twist& t) {
			this->Set(t);
		}),
		m_enabled(false)
{
	nh.subscribe(m_sub);
}
void DiffDriveController::Set(const geometry_msgs::Twist& t)
{
	Set(t.linear.x, t.angular.z);
}
void DiffDriveController::Set(double linear, double angular)
{
	if (!m_enabled) return;
	double left = linear - (angular * m_separation / 2.0);
	double right = linear + (angular * m_separation / 2.0);
	m_left->SetSetpoint(left);
	m_right->SetSetpoint(right);
}
bool DiffDriveController::getEnabled() { return m_enabled;}
void DiffDriveController::Disable()
{
	m_left->Disable();
	m_right->Disable();
	m_enabled = false;
}
void DiffDriveController::Enable() {
	m_left->Enable();
	m_right->Enable();
	m_left->SetSetpoint(0);
	m_right->SetSetpoint(0);
	m_enabled = true;
}
