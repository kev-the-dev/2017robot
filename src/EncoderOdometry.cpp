#include "EncoderOdometry.hpp"
#ifdef USE_ROS
EncoderOdometry::EncoderOdometry(ros::NodeHandle& nh, const char* topic, std::shared_ptr<frc::Encoder> left_encoder, std::shared_ptr<frc::Encoder> right_encoder, double wheel_sep):
		m_left(left_encoder),
		m_right(right_encoder),
		odom_pub(topic, &odom_msg),
		first_update(true),
		wheel_seperation(wheel_sep),
		m_nh(nh),
		x(0.0), y(0.0), th(0.0),
		last_left(0.0), last_right(0.0)
#else
EncoderOdometry::EncoderOdometry(const char* topic, std::shared_ptr<frc::Encoder> left_encoder, std::shared_ptr<frc::Encoder> right_encoder, double wheel_sep):
		m_left(left_encoder),
		m_right(right_encoder),
		first_update(true),
		wheel_seperation(wheel_sep),
		x(0.0), y(0.0), th(0.0),
		last_left(0.0), last_right(0.0)
#endif
{
	odom_msg.header.frame_id = "odom";
	odom_msg.child_frame_id = "base_link";

	for (size_t i = 0; i < 36; i++)
	{
		odom_msg.pose.covariance[i] = 0.0;
		odom_msg.twist.covariance[i] = 0.0;
	}

	odom_msg.pose.covariance[0]  = 0.01;       // X
	odom_msg.pose.covariance[7]  = 0.01;       // Y
	odom_msg.pose.covariance[14] = 1000000.0;  // Z
	odom_msg.pose.covariance[21] = 1000000.0;  // Rotation X
	odom_msg.pose.covariance[28] = 1000000.0;  // Rotation Y
	odom_msg.pose.covariance[36] = 0.01;       // Rotation Z

	odom_msg.twist.covariance[0]  = 0.01;       // X
	odom_msg.twist.covariance[7]  = 0.01;       // Y
	odom_msg.twist.covariance[14] = 1000000.0;  // Z
	odom_msg.twist.covariance[21] = 1000000.0;  // Rotation X
	odom_msg.twist.covariance[28] = 1000000.0;  // Rotation Y
	odom_msg.twist.covariance[36] = 0.01;       // Rotation Z

	odom_transform.header.frame_id = "odom";
	odom_transform.child_frame_id = "base_link";

#ifdef USE_ROS
	tf_broadcaster.init(nh);
	nh.advertise(odom_pub);
#endif
}
void EncoderOdometry::update()
{
	if (first_update)
	{
		//timer.Start();
		//last_time = std::chrono::high_resolution_clock::now();
		Reset();
		first_update = false;
	}
	double dt = timer.Get();
	timer.Reset();
	//std::chrono::time_point<std::chrono::high_resolution_clock> t = std::chrono::high_resolution_clock::now();
	//double dt = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(t - last_time)).count();
	//last_time = t;
	double cur_left = m_left->GetDistance();
	double cur_right = m_right->GetDistance();
	double d_left = cur_left - last_left;
	double d_right = cur_right - last_right;
	last_left = cur_left;
	last_right = cur_right;
	double d_mag = (d_left+d_right)/2.0;

	double delta_th = (d_right - d_left) / wheel_seperation;
	double delta_x  = d_mag * std::cos(th);
	double delta_y  = d_mag * std::sin(th);

	double vx = delta_x / dt;
	double vy = delta_y / dt;
	double vth = delta_th / dt;

	x += delta_x;
	y += delta_y;
	th += delta_th;

	odom_msg.pose.pose.position.x = x;
	odom_msg.pose.pose.position.y = y;
	odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(th);
	odom_msg.twist.twist.linear.x = vx;
	odom_msg.twist.twist.linear.y = vy;
	odom_msg.twist.twist.angular.z = vth;

	odom_transform.header.stamp = odom_msg.header.stamp;
	odom_transform.transform.translation.x = odom_msg.pose.pose.position.x;
	odom_transform.transform.translation.y = odom_msg.pose.pose.position.y;
	odom_transform.transform.rotation = odom_msg.pose.pose.orientation;

#ifdef USE_ROS
	odom_msg.header.stamp = m_nh.now();
	odom_pub.publish(&odom_msg);
	tf_broadcaster.sendTransform(odom_transform);
#endif
}
void EncoderOdometry::Reset()
{
	Reset(0,0,0);
}
void EncoderOdometry::Reset(double x_i, double y_i, double th_i)
{
	x = x_i;
	y = y_i;
	th = th_i;
	last_left = m_left->GetDistance();
	last_right = m_right->GetDistance();
	//last_time = std::chrono::high_resolution_clock::now();
	//timer.Stop();
	timer.Reset();
	//timer.Start();
}
nav_msgs::Odometry EncoderOdometry::Get()
{
	return odom_msg;
}
double EncoderOdometry::GetThetaDeg()
{
	return th*(180.0/PI);
}
double EncoderOdometry::GetX()
{
	return x;
}
double EncoderOdometry::GetY()
{
	return y;
}
