#pragma once

#include <rosfrc/RosRobot.h>
#include <Encoder.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include "UseRos.h"


#ifdef USE_ROS
class EncoderOdometry: public rosfrc::Updater
#else
class EncoderOdometry
#endif
{
private:
	const double PI = 3.14159265359;
	std::shared_ptr<frc::Encoder> m_left;
	std::shared_ptr<frc::Encoder> m_right;
	nav_msgs::Odometry odom_msg;
	geometry_msgs::TransformStamped odom_transform;
#ifdef USE_ROS
	tf::TransformBroadcaster tf_broadcaster;
	ros::Publisher odom_pub;
	ros::NodeHandle& m_nh;
#endif
	//std::chrono::time_point<std::chrono::high_resolution_clock> last_time;
	frc::Timer timer;
	bool first_update;
	double wheel_seperation;
	double x, y, th;
	double last_left, last_right;
public:
#ifdef USE_ROS
	EncoderOdometry(ros::NodeHandle& nh, const char* topic, std::shared_ptr<frc::Encoder> left_encoder, std::shared_ptr<frc::Encoder> right_encoder, double wheel_sep);
#else
	EncoderOdometry(const char* topic, std::shared_ptr<frc::Encoder> left_encoder, std::shared_ptr<frc::Encoder> right_encoder, double wheel_sep);
#endif
	virtual void update();
	void Reset();
	void Reset(double x_i, double y_i, double th_i);
	nav_msgs::Odometry Get();
	double GetThetaDeg();
	double GetX();
	double GetY();
	double GetTheta();
	virtual ~EncoderOdometry(){};
};
