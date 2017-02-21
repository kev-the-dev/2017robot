#pragma once

#include <memory>
#include <rosfrc/RosRobot.h>
#include <geometry_msgs/Twist.h>
#include <PIDController.h>
#include "UseRos.h"

const double PI = 3.14159265359;

class DiffDriveController
{
private:
	std::shared_ptr<frc::PIDController> m_left;
	std::shared_ptr<frc::PIDController> m_right;
	double m_separation;
#ifdef USE_ROS
	ros::Subscriber<geometry_msgs::Twist> m_sub;
#endif
	bool m_enabled;
public:
#ifdef USE_ROS
	DiffDriveController(ros::NodeHandle& nh, char* topic, std::shared_ptr<frc::PIDController> left, std::shared_ptr<frc::PIDController> right, double speration);
#else
	DiffDriveController(char* topic, std::shared_ptr<frc::PIDController> left, std::shared_ptr<frc::PIDController> right, double speration);
#endif
	void Set(const geometry_msgs::Twist& t);
	void Set(double linear, double angular);
	void Disable();
	void Enable();
	bool getEnabled();
};
