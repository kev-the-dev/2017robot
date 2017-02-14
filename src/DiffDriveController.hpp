#pragma once

#include <memory>
#include <rosfrc/RosRobot.h>
#include <geometry_msgs/Twist.h>
#include <PIDController.h>

const double PI = 3.14159265359;

class DiffDriveController
{
private:
	std::shared_ptr<frc::PIDController> m_left;
	std::shared_ptr<frc::PIDController> m_right;
	double m_separation;
	ros::Subscriber<geometry_msgs::Twist> m_sub;
	bool m_enabled;
public:
	DiffDriveController(ros::NodeHandle& nh, char* topic, std::shared_ptr<frc::PIDController> left, std::shared_ptr<frc::PIDController> right, double speration);
	void Set(const geometry_msgs::Twist& t);
	void Disable();
	void Enable();
	bool getEnabled();
};
