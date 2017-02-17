
// Ros stuff
#include <rosfrc/RosRobot.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include "EncoderOdometry.hpp"
#include "DiffDriveController.hpp"
#include "BaseServo.h"

// FRC Stuff
#include <RobotDrive.h>
#include <VictorSP.h>
#include <Encoder.h>
#include <Servo.h>
#include <BuiltInAccelerometer.h>
#include <interfaces/Gyro.h>
#include <ADXRS450_Gyro.h>
#include <CameraServer.h>
#include <Counter.h>
#include <PIDController.h>
#include <SpeedController.h>
#include <Servo.h>
#include <memory>
#include <cmath>
#include <DigitalInput.h>

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

class TwoMotorOutput: public frc::PIDOutput
{
private:
	std::shared_ptr<frc::SpeedController> m_one;
	std::shared_ptr<frc::SpeedController> m_two;
public:
	TwoMotorOutput(std::shared_ptr<frc::SpeedController> one, std::shared_ptr<frc::SpeedController> two);
	virtual ~TwoMotorOutput();
	void PIDWrite(double output) override;
	void Set(double output);
};
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

#define KEVIN_LAPTOP_IP "10.41.18.94:5802"
#define KANGAROO_IP     "10.41.18.24:5802"

//#define USE_VEL_TELEOP

/* CONTROLS:
 * -------Joystick 1: (stick left)-------
 * XY axes - drive forward/backward
 * XY and hold trigger - drive slower forward/backward
 * Button 4 - climb while held
 * -------Joystick 2: (stick middle)-------
 * XY axes - rotate
 * XY and hold trigger - slow rotate
 * Button 3 - push claws out and auto retract
 */


class Robot: public rosfrc::RosRobot {
private:
	const double WHEEL_RADIUS_METERS = 0.0762;
	const double CYCLES_PER_REVOLUTION = 360.0;
	const double WHEEL_SEPERATION_METERS = 0.6096;
	const double ENCODER_REVOLUTIONS_PER_OUTPUT_REVOLUTION = 11.25;
	const double MAX_LINEAR_VELOCITY_METERS_PER_SECOND = 5.4;
	const double MAX_ANGULAR_VELOCITY = 6.0;

	const double WHEEL_CIRCUMFERENCE_METERS = 2.0 * PI * WHEEL_RADIUS_METERS;
	const double DISTANCE_PER_CYCLE_METERS = (WHEEL_CIRCUMFERENCE_METERS / CYCLES_PER_REVOLUTION) / ENCODER_REVOLUTIONS_PER_OUTPUT_REVOLUTION;
	const double DRIVE_CONTROLLER_KF = 1.0 / MAX_LINEAR_VELOCITY_METERS_PER_SECOND;
public:
	std::shared_ptr<frc::Joystick> stickLeft;
	std::shared_ptr<frc::Joystick> stickMiddle;
	std::shared_ptr<frc::Joystick> stickRight;
	std::shared_ptr<frc::VictorSP> BL;
	std::shared_ptr<frc::VictorSP> BR;
	std::shared_ptr<frc::VictorSP> FL;
	std::shared_ptr<frc::VictorSP> FR;
	std::shared_ptr<frc::VictorSP> climberOne;
	std::shared_ptr<frc::VictorSP> climberTwo;
	TwoMotorOutput climber;
	std::shared_ptr<BuiltInAccelerometer> accel;
	std::shared_ptr<frc::Encoder> encoder_left_front;
	std::shared_ptr<frc::Encoder> encoder_right_front;
	//std::shared_ptr<frc::Gyro> gyro;
	frc::RobotDrive drive;
	TwoMotorOutput left_motors;
	TwoMotorOutput right_motors;
	float p, i ,d;
	std::shared_ptr<frc::PIDController> left_controller;
	std::shared_ptr<frc::PIDController> right_controller;
	std::string str;
	EncoderOdometry odom;
	DiffDriveController ctrl;
	std::shared_ptr<BaseServo> servo_left;
	std::shared_ptr<BaseServo> servo_right;
	Robot():
			RosRobot(KANGAROO_IP),
			stickLeft(new frc::Joystick(0)),
			stickMiddle(new frc::Joystick(1)),
			stickRight(new frc::Joystick(2)),
			BL(new VictorSP(0)),
			BR(new VictorSP(2)),
			FL(new VictorSP(1)),
			FR(new VictorSP(3)),
			climberOne(new VictorSP(4)),
			climberTwo(new VictorSP(5)),
			climber(climberOne, climberTwo),
			accel(new BuiltInAccelerometer()),
			encoder_left_front(new Encoder(7 ,8, false, frc::Encoder::k4X)),
			encoder_right_front(new Encoder(5, 6,false, frc::Encoder::k4X)),
			//gyro(new ADXRS450_Gyro()),
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
			servo_left(new ServoHS805BB(6)),
			servo_right(new ServoHS805BB(7))
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
		servo_left->SetSafetyEnabled(false);
		servo_right->SetSafetyEnabled(false);
		AddUpdater(&odom);
		AddEncoder("/encoder_left", encoder_left_front);
		AddEncoder("/encoder_right", encoder_right_front);
		//getNodeHandle().advertise(cPub);
	}
	void MoveClawsOut(){
		servo_left->SetAngle(0);
		servo_right->SetAngle(95);
	}
	void MoveClawsIn(){
		servo_left->SetAngle(95);
		servo_right->SetAngle(0);
	}
	void AutonomousInit() override
	{
		odom.Reset();
		getNodeHandle().loginfo("Auto Begin");
	}
	void AutonomousPeriodic() override
	{

	}

	void TeleopInit() override
	{
#ifdef USE_VEL_TELEOP
		ctrl.Enable();
#else
		ctrl.Disable();
#endif
		getNodeHandle().loginfo("Teleop Begin");
		MoveClawsIn();
	}
	void TeleopPeriodic() override
	{
#ifdef USE_VEL_TELEOP
		geometry_msgs::Twist twist;
		twist.linear.x = -stickLeft->GetY()*MAX_LINEAR_VELOCITY_METERS_PER_SECOND;
		twist.angular.z = -stickMiddle->GetX()*MAX_ANGULAR_VELOCITY;
		ctrl.Set(twist);
#else
		bool slow = stickLeft->GetRawButton(1);
		int s = 1;
		double x = stickLeft->GetY();
		double rotation = stickMiddle->GetX();
		if (slow) s = 2;
		drive.ArcadeDrive(rotation/s, x/s);
#endif
		if (stickLeft->GetRawButton(4)) climber.Set(-1);
		else if (stickLeft->GetRawButton(5)) climber.Set(-0.25);
		else climber.Set(0);
		if (stickMiddle->GetRawButton(3)) MoveClawsOut();
		else MoveClawsIn();
}
	void DisabledInit() override
	{
		getNodeHandle().loginfo("Disabled");

	}
	void DisabledPeriodic() override
	{

	}
	void TestInit() override
	{
		getNodeHandle().loginfo("Test Begin");
		//ctrl.Enable();
		//geometry_msgs::Twist twist_msg;
		//twist_msg.linear.x = 1.5;
		//ctrl.Set(twist_msg);
		FL->Set(0.5);
		FR->Set(0.5);
		BL->Set(0.5);
		BR->Set(0.5);

	}
	void TestPeriodic() override
	{

	}
	void RobotPeriodic () override
	{

	}
};

START_ROBOT_CLASS(Robot)
