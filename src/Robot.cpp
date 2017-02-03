
// Ros stuff
#include <rosfrc/RosRobot.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

// FRC Stuff
#include <RobotDrive.h>
#include <Talon.h>
#include <Victor.h>
#include <Encoder.h>
#include <Servo.h>
#include <BuiltInAccelerometer.h>
#include <interfaces/Gyro.h>
#include <ADXRS450_Gyro.h>
#include <CameraServer.h>
#include <Counter.h>
#include <PIDController.h>
#include <SpeedController.h>
#include <I2C.h>
#include <memory>
#include <cmath>

#include <thread>
#include <chrono>

/*
constexpr const char* OnboardResourceVISA = "ASRL1::INSTR";
constexpr const char* MxpResourceVISA = "ASRL2::INSTR";

constexpr const char* OnboardResourceOS = "/dev/ttyS0";
constexpr const char* MxpResourceOS = "/dev/ttyS1";

 */

/*
 * white = FL = 4
 * blACK = fr = 5
 * gray = bl = 6
 * PURPLE = br = 7
 *
 * DO0 = BL
 * DO2 = FR
 * DO1 = FL
 * DO3 = BR
 *
 */
/* E4P encoders
 * 360 cycles per revoluton, 1440 pules including 4x
 *
 *
 *
 *
 */

class LidarLite
{
private:
	enum VALUES {
		RESET = 0x00,
		MEASURE_UNCORRECTED = 0x03,
		MEASURE_CORRECTED = 0x04
	};
	enum REGISTERS {
		STATUS = 0x01,
		ACQ_COMMAND = 0x00,
		DISTANCE = 0x08,
		SIG_COUNT_VAL = 0x02,
		ACQ_CONFIG_REG = 0x04,
		THRESHOLD_BYPASS = 0x1c
	};
	const uint8_t STATUS_REGISTER = 0x01;
	const unsigned int TIMEOUT_MILISECONDS = 200;
	bool WaitForAvailable();
	bool Busy();
	frc::I2C device;
public:
	LidarLite(frc::I2C::Port port=frc::I2C::Port::kOnboard, int device=0x62);
	bool SetAddress(uint8_t serialnumber, int address);
	bool Reset();
	bool configure(int config=0);
	int GetDistance(bool corrected=true);

};
bool LidarLite::WaitForAvailable()
{
	auto timeout = std::chrono::steady_clock::now() + std::chrono::milliseconds(TIMEOUT_MILISECONDS);
	while (std::chrono::steady_clock::now() < timeout)
	{
		if (!Busy()) return true;
	}
	printf("Wait for not busy timed out line %d\n", __LINE__);
	return false;
}
bool LidarLite::Busy()
{
	uint8_t statusWrite = REGISTERS::STATUS;
	uint8_t statusRead = 0;
	if(device.WriteBulk(&statusWrite, 1)) printf("Write failed at %d", __LINE__);
	if(device.ReadOnly(1, &statusRead)) printf("Read failed at %d", __LINE__);
	return statusRead & 0x01;
}
LidarLite::LidarLite(frc::I2C::Port port, int device) :
		device(port, device)
{

}
/*------------------------------------------------------------------------------
  Configure

  Selects one of several preset configurations.

  Parameters
  ------------------------------------------------------------------------------
  configuration:  Default 0.
    0: Default mode, balanced performance.
    1: Short range, high speed. Uses 0x1d maximum acquisition count.
    2: Default range, higher speed short range. Turns on quick termination
        detection for faster measurements at short range (with decreased
        accuracy)
    3: Maximum range. Uses 0xff maximum acquisition count.
    4: High sensitivity detection. Overrides default valid measurement detection
        algorithm, and uses a threshold value for high sensitivity and noise.
    5: Low sensitivity detection. Overrides default valid measurement detection
        algorithm, and uses a threshold value for low sensitivity and noise.
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
bool LidarLite::configure(int configuration)
{
	  switch (configuration)
	  {
	    case 0: // Default mode, balanced performance
	      device.Write(0x02,0x80); // Default
	      device.Write(0x04,0x08); // Default
	      device.Write(0x1c,0x00); // Default
	    break;

	    case 1: // Short range, high speed
	      device.Write(0x02,0x1d);
	      device.Write(0x04,0x08); // Default
	      device.Write(0x1c,0x00); // Default
	    break;

	    case 2: // Default range, higher speed short range
	      device.Write(0x02,0x80); // Default
	      device.Write(0x04,0x00);
	      device.Write(0x1c,0x00); // Default
	    break;

	    case 3: // Maximum range
	      device.Write(0x02,0xff);
	      device.Write(0x04,0x08); // Default
	      device.Write(0x1c,0x00); // Default
	    break;

	    case 4: // High sensitivity detection, high erroneous measurements
	      device.Write(0x02,0x80); // Default
	      device.Write(0x04,0x08); // Default
	      device.Write(0x1c,0x80);
	    break;

	    case 5: // Low sensitivity detection, low erroneous measurements
	      device.Write(0x02,0x80); // Default
	      device.Write(0x04,0x08); // Default
	      device.Write(0x1c,0xb0);
	    break;
	  }
	  return false;
}
bool LidarLite::SetAddress(uint8_t serialnumber, int address)
{
	uint8_t res[2];
	if (device.Read(0x16, 1, &res[0])) return true; // Get Serial high byte
	if (device.Read(0x17, 1, &res[1])) return true; // Get serial low byte
	if (device.Write(0x18, res[0])) return true;   // Write first byte of serial number
	if (device.Write(0x19, res[1])) return true;   // Write second bye of serial number
	if (device.Write(0x1a, address)) return true;
	if (device.Write(0x1e, 0x08)) return true;
	return false;
}
bool LidarLite::Reset()
{
	return device.Write(REGISTERS::ACQ_COMMAND, VALUES::RESET);
}
int LidarLite::GetDistance(bool corrected)
{
	if(!WaitForAvailable()) return -1;
	if (corrected)
	{
		//if (device.Write(0x00, 0x04)) return -1;
		if(device.Write(REGISTERS::ACQ_COMMAND, VALUES::MEASURE_CORRECTED))
		{
			printf("Write failed at line %d\n", __LINE__);
			return -1;
		}
	}
	else
	{
		if (device.Write(REGISTERS::ACQ_COMMAND, VALUES::MEASURE_UNCORRECTED))
		{
			printf("Write failed at line %d\n", __LINE__);
			return -1;
		}
	}
	if(!WaitForAvailable()) return -1;
	uint8_t readRegister = REGISTERS::DISTANCE;
	if(device.WriteBulk(&readRegister, 1)) printf("Write failed at %d\n", __LINE__);
	uint8_t distanceRead[2];
	if(device.ReadOnly(2, distanceRead)) printf("Read failed at line %d\n", __LINE__);
	unsigned int distance = (unsigned int)(distanceRead[0]<<8) + (unsigned int)(distanceRead[1]);
	return distance;
}

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
	if (!m_enabled) return;
	double linear = t.linear.x;
	double angular = t.angular.z;
	double left = linear - (angular * m_separation / 2.0);
	double right = linear + (angular * m_separation / 2.0);
	m_left->SetSetpoint(left);
	m_right->SetSetpoint(right);
}
bool DiffDriveController::getEnabled() { return m_enabled;}
void DiffDriveController::Disable() { m_enabled = false; }
void DiffDriveController::Enable() { m_enabled = true; }

class EncoderOdometry: public rosfrc::Updater
{
private:
	std::shared_ptr<frc::Encoder> m_left;
	std::shared_ptr<frc::Encoder> m_right;
	nav_msgs::Odometry odom_msg;
	geometry_msgs::TransformStamped odom_transform;
	tf::TransformBroadcaster tf_broadcaster;
	ros::Publisher odom_pub;
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
	virtual ~EncoderOdometry(){};
};
EncoderOdometry::EncoderOdometry(ros::NodeHandle& nh, const char* topic, std::shared_ptr<frc::Encoder> left_encoder, std::shared_ptr<frc::Encoder> right_encoder, double wheel_sep):
		m_left(left_encoder),
		m_right(right_encoder),
		odom_pub(topic, &odom_msg),
		first_update(true),
		wheel_seperation(wheel_sep),
		m_nh(nh),
		x(0.0), y(0.0), th(0.0),
		last_left(0.0), last_right(0.0)
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

	tf_broadcaster.init(nh);
	nh.advertise(odom_pub);
}
void EncoderOdometry::update()
{
	if (first_update)
	{
		Reset();
		first_update = false;
	}
	double dt = timer.Get();
	timer.Reset();
	double cur_left = m_left->GetDistance();
	double cur_right = m_right->GetDistance();
	double v_left = cur_left - last_left;
	double v_right = cur_right - last_right;
	last_left = cur_left;
	last_right = cur_right;
	double v_mag = (v_left+v_right)/2.0;

	double vx = v_mag * std::cos(th);
	double vy = v_mag * std::sin(th);
	double vth = (v_right - v_left) / wheel_seperation;

	double delta_th =  vth * dt;
	double delta_x  =  vx * dt;
	double delta_y  =  vy * dt;

	x += delta_x;
	y += delta_y;
	th += delta_th;

	odom_msg.header.stamp = m_nh.now();
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

	odom_pub.publish(&odom_msg);
	tf_broadcaster.sendTransform(odom_transform);
}
void EncoderOdometry::Reset()
{
	x = 0;
	y = 0;
	th = 0;
	last_left = m_left->GetDistance();
	last_right = m_right->GetDistance();
	timer.Stop();
	timer.Reset();
	timer.Start();
}
void EncoderOdometry::Reset(double x_i, double y_i, double th_i)
{
	x = x_i;
	y = y_i;
	th = th_i;
	last_left = m_left->GetDistance();
	last_right = m_right->GetDistance();
	timer.Stop();
	timer.Reset();
	timer.Start();
}

class TwoMotorOutput: public frc::PIDOutput
{
private:
	std::shared_ptr<frc::SpeedController> m_one;
	std::shared_ptr<frc::SpeedController> m_two;
public:
	TwoMotorOutput(std::shared_ptr<frc::SpeedController> one, std::shared_ptr<frc::SpeedController> two);
	virtual ~TwoMotorOutput();
	void PIDWrite(double output) override;
};
TwoMotorOutput::TwoMotorOutput(std::shared_ptr<frc::SpeedController> one, std::shared_ptr<frc::SpeedController> two):
		m_one(one),
		m_two(two)
{

}
TwoMotorOutput::~TwoMotorOutput(){};
void TwoMotorOutput::PIDWrite(double output)
{
	m_one->Set(output);
	m_two->Set(output);
}

//#define USE_VEL_TELEOP

class Robot: public rosfrc::RosRobot {
private:
	const double WHEEL_RADIUS_METERS = 0.0762;
	const double WHEEL_CIRCUMFERENCE_METERS = 2.0 * PI * WHEEL_RADIUS_METERS;
	const double CYCLES_PER_REVOLUTION = 360.0;
	const double ENCODER_REVOLUTIONS_PER_OUTPUT_REVOLUTION = 2;
	const double DISTANCE_PER_CYCLE_METERS = (WHEEL_CIRCUMFERENCE_METERS / CYCLES_PER_REVOLUTION) * ENCODER_REVOLUTIONS_PER_OUTPUT_REVOLUTION;
	const double MAX_LINEAR_VELOCITY_METERS_PER_SECOND = 5.4;
	const double MAX_ANGULAR_VELOCITY = 6.0;
	const double DRIVE_CONTROLLER_KF = 1.0 / MAX_LINEAR_VELOCITY_METERS_PER_SECOND;
	const double WHEEL_SEPERATION_METERS = 0.3048;
public:
	std::shared_ptr<frc::Joystick> stickLeft;
	std::shared_ptr<frc::Joystick> stickMiddle;
	std::shared_ptr<frc::Joystick> stickRight;
	std::shared_ptr<frc::Talon> BL;
	std::shared_ptr<frc::Talon> BR;
	std::shared_ptr<frc::Talon> FL;
	std::shared_ptr<frc::Talon> FR;
	std::shared_ptr<frc::Victor> TestOne;
	std::shared_ptr<frc::Victor> TestTwo;
	std::shared_ptr<BuiltInAccelerometer> accel;
	std::shared_ptr<frc::Encoder> encoder_left_front;
	std::shared_ptr<frc::Encoder> encoder_right_front;
	std::shared_ptr<frc::Gyro> gyro;
	frc::RobotDrive drive;
	TwoMotorOutput left_motors;
	TwoMotorOutput right_motors;
	float p, i ,d;
	std::shared_ptr<frc::PIDController> left_controller;
	std::shared_ptr<frc::PIDController> right_controller;
	std::string str;
	EncoderOdometry odom;
	DiffDriveController ctrl;
	Servo servo;
	//Counter c;
	LidarLite ll;
	Robot():
			RosRobot("10.41.18.94"),
			stickLeft(new frc::Joystick(0)),
			stickMiddle(new frc::Joystick(1)),
			stickRight(new frc::Joystick(2)),
			BL(new Talon(3)),
			BR(new Talon(2)),
			FL(new Talon(0)),
			FR(new Talon(1)),
			TestOne(new Victor(4)),
			TestTwo(new Victor(5)),
			accel(new BuiltInAccelerometer()),
			encoder_left_front(new Encoder(7, 8)),
			encoder_right_front(new Encoder(2, 5)),
			gyro(new ADXRS450_Gyro()),
			drive(*FL, *BL, *FR, *BR),
			left_motors(BL, FL),
			right_motors(BR, FR),
			p(0.1),
			i(0.0),
			d(0.325),
			left_controller(new PIDController(p, i, d, DRIVE_CONTROLLER_KF, encoder_left_front.get(), &left_motors)),
			right_controller(new PIDController(p, i, d, DRIVE_CONTROLLER_KF, encoder_right_front.get(), &right_motors)),
			odom(getNodeHandle(), "/odom", encoder_left_front, encoder_right_front, WHEEL_SEPERATION_METERS),
			ctrl(getNodeHandle(), "/cmd_vel", left_controller, right_controller, WHEEL_SEPERATION_METERS),
			servo(6),
			//c(7),
			ll(frc::I2C::Port::kMXP, 0x62)
	{
		encoder_left_front->SetDistancePerPulse(DISTANCE_PER_CYCLE_METERS);
		encoder_left_front->SetPIDSourceType(PIDSourceType::kRate);
		encoder_left_front->SetReverseDirection(true);
		encoder_right_front->SetDistancePerPulse(DISTANCE_PER_CYCLE_METERS);
		encoder_right_front->SetPIDSourceType(PIDSourceType::kRate);
		left_controller->SetInputRange(-MAX_LINEAR_VELOCITY_METERS_PER_SECOND, MAX_LINEAR_VELOCITY_METERS_PER_SECOND);
		right_controller->SetInputRange(-MAX_LINEAR_VELOCITY_METERS_PER_SECOND, MAX_LINEAR_VELOCITY_METERS_PER_SECOND);
		BL->SetSafetyEnabled(false);
		BR->SetSafetyEnabled(false);
		FL->SetSafetyEnabled(false);
		FR->SetSafetyEnabled(false);
		drive.SetSafetyEnabled(false);
		FR->SetInverted(true);
		BR->SetInverted(true);
		/*
		AddJoystick("/joysticks/left", stickLeft);
		AddJoystick("/joysticks/middle", stickMiddle);
		AddSpeedController("/speed_controllers/FL", FL);
		AddSpeedController("/speed_controllers/FR", FR);
		AddSpeedController("/speed_controllers/BL", BL);
		AddSpeedController("/speed_controllers/BR", BR);
		AddSpeedController("/speed_controllers/TestOne", TestOne);
		AddSpeedController("/speed_controllers/TestTwo", TestTwo);
		AddEncoder("/encoder_right", encoder_right_front);
		AddPIDController("/left_wheels_vel", left_controller);
		AddPIDController("/right_wheels_vel", right_controller)
		AddAccelerometer("/accel", accel);
		AddGyro("/gyro", gyro);
		*/
		//ll.configure(1);
		AddUpdater(&odom);
		AddEncoder("/encoder_left", encoder_left_front);
	}

	void AutonomousInit() override
	{
		getNodeHandle().loginfo("Auto Begin");

		left_controller->SetPID(p, i, d);
		right_controller->SetPID(p, i, d);
		using namespace std;
		std::stringstream s;

		left_controller->Reset();
		right_controller->Reset();
		left_controller->SetSetpoint(0);
		right_controller->SetSetpoint(0);

		s << "P=" << left_controller->GetP() << " I=" << left_controller->GetI() << " D=" << left_controller->GetD() << "Setpoint = " << left_controller->GetSetpoint();
		getNodeHandle().loginfo(s.str().c_str());

		ctrl.Enable();
		left_controller->Enable();
		right_controller->Enable();
	}
	void AutonomousPeriodic() override
	{
		std::stringstream s;
		s << "SETPOINT= " << left_controller->GetSetpoint() << "LEFT OUTPUT: "  << left_controller->Get() << "\n RIGHT OUTPUT: " << right_controller->Get();
		getNodeHandle().loginfo(s.str().c_str());
	}

	void TeleopInit() override
	{
#ifdef USE_VEL_TELEOP
		left_controller->Reset();
		right_controller->Reset();
		left_controller->SetSetpoint(0);
		right_controller->SetSetpoint(0);
		ctrl.Enable();

		std::stringstream s;
		s << "P=" << left_controller->GetP() << " I=" << left_controller->GetI() << " D=" << left_controller->GetD() << "Setpoint = " << left_controller->GetSetpoint();
		getNodeHandle().loginfo(s.str().c_str());
#else
		left_controller->Disable();
		right_controller->Disable();
		ctrl.Disable();
#endif
		//odom.Reset();
		getNodeHandle().loginfo("Teleop Begin");
	}
	void TeleopPeriodic() override
	{
#ifdef USE_VEL_TELEOP
		geometry_msgs::Twist twist;
		twist.linear.x = -stickLeft->GetY()*MAX_LINEAR_VELOCITY_METERS_PER_SECOND;
		twist.angular.z = -stickMiddle->GetX()*MAX_ANGULAR_VELOCITY;
		ctrl.Set(twist);
#else
		double x = stickLeft->GetY();
		double rotate = stickMiddle->GetX();
		drive.ArcadeDrive(rotate, x);
#endif
	}
	void DisabledInit() override
	{
		getNodeHandle().loginfo("Disabled");
		left_controller->Disable();
		right_controller->Disable();
	}
	void DisabledPeriodic() override
	{

	}
	void TestInit() override
	{
		ctrl.Disable();
		left_controller->Disable();
		right_controller->Disable();
	}
	void TestPeriodic() override
	{
		/*
		double speed = stickRight->GetY();
		TestOne->Set(speed);
		TestTwo->Set(speed);
		*/
		std::stringstream s;
		//s << "Counter:" << c.Get() << std::endl;
		//getNodeHandle().loginfo(s.str().c_str());
		double servo_scale = stickRight->GetZ();
		servo.Set(servo_scale);
	}
	void RobotPeriodic () override
	{
		std::stringstream s;
		int ping = ll.GetDistance();
		if (ping < 0) s << "Error reading lidar";
		else s << "Lidar: " << ping;
		getNodeHandle().loginfo(s.str().c_str());
	}
};

START_ROBOT_CLASS(Robot)
