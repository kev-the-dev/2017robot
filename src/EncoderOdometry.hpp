#pragma once

#include <rosfrc/RosRobot.h>
#include <Encoder.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

class EncoderOdometry: public rosfrc::Updater
{
private:
	std::shared_ptr<frc::Encoder> m_left;
	std::shared_ptr<frc::Encoder> m_right;
	nav_msgs::Odometry odom_msg;
	geometry_msgs::TransformStamped odom_transform;
	tf::TransformBroadcaster tf_broadcaster;
	ros::Publisher odom_pub;
	//std::chrono::time_point<std::chrono::high_resolution_clock> last_time;
	frc::Timer timer;
	bool first_update;
	double wheel_seperation;
	ros::NodeHandle& m_nh;
	double x, y, th;
	double last_left, last_right;
public:
	EncoderOdometry(ros::NodeHandle& nh, const char* topic, std::shared_ptr<frc::Encoder> left_encoder, std::shared_ptr<frc::Encoder> right_encoder, double wheel_sep);
	virtual void update();
	void Reset();
	void Reset(double x_i, double y_i, double th_i);
	nav_msgs::Odometry Get();
	virtual ~EncoderOdometry(){};
};
