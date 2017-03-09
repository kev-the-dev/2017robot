
// Ros stuff
#include <rosfrc/RosRobot.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

// Other files in this project
#include "EncoderOdometry.hpp"
#include "DiffDriveController.hpp"
#include "BaseServo.h"
#include "TwoMotorOutput.hpp"
#include "Commands/DoNothing.h"
#include "Commands/DriveForwardAuto.h"
#include "Commands/CenterGear.h"
#include "Commands/EncoderAuto.h"
#include "Commands/SideGearAuto.h"

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
#include <SmartDashboard/SendableChooser.h>
#include "RobotMap.h"
#include "UseRos.h"

#include <thread>
#include <chrono>

#ifdef USE_ROS
#define KEVIN_LAPTOP_IP "10.41.18.94:5802"
#define KANGAROO_IP     "10.41.18.24:5802"
#endif

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


#ifdef USE_ROS
class Robot: public rosfrc::RosRobot, RobotMap {
#else
	class Robot: public frc::IterativeRobot, RobotMap {
#endif
public:
	SendableChooser<std::shared_ptr<Command>> auto_chooser;
	std::shared_ptr<Command> autoCommand;
	cs::UsbCamera cam;
	double max_vel;
	Robot()
#ifdef USE_ROS
			:
				RosRobot(KEVIN_LAPTOP_IP)
#else

#endif
	{
#ifdef USE_ROS
		RobotMap::init(getNodeHandle());
		AddUpdater(odom.get());
		AddEncoder("/encoder_left", encoder_left);
		AddEncoder("/encoder_right", encoder_right);
#else
		RobotMap::init();
#endif

		auto_chooser.AddDefault("Do Nothing",std::shared_ptr<Command>(new DoNothing()));
		auto_chooser.AddObject ("Move Forward", std::shared_ptr<Command>(new DriveForwardAuto()));
//		auto_chooser.AddObject ("Center Gear", std::shared_ptr<Command>(new CenterGear(0)));
//		auto_chooser.AddObject ("Center Gear Left", std::shared_ptr<Command>(new CenterGear(-1)));
//		auto_chooser.AddObject ("Center Gear Right", std::shared_ptr<Command>(new CenterGear(1)));
		auto_chooser.AddObject ("Center Gear", std::shared_ptr<Command>(new EncoderAuto(0)));
		auto_chooser.AddObject ("Center Gear Left", std::shared_ptr<Command>(new EncoderAuto(-1)));
		auto_chooser.AddObject ("Center Gear Right", std::shared_ptr<Command>(new EncoderAuto(1)));
		auto_chooser.AddObject ("Side Gear Left", std::shared_ptr<Command>(new SideGearAuto(false)));
		auto_chooser.AddObject ("Side Gear Right", std::shared_ptr<Command>(new SideGearAuto(true)));
		SmartDashboard::PutData("Auto Program", &auto_chooser);

		SmartDashboard::PutNumber("P", RobotMap::P);
		SmartDashboard::PutNumber("D", RobotMap::D);
		max_vel = 0.0;
	}
	void RobotInit() override
	{
		cam = CameraServer::GetInstance()->StartAutomaticCapture(0);
	}
	void AutonomousInit() override
	{
#ifdef USE_ROS
		getNodeHandle().loginfo("Auto Begin");
#endif
		//TestInit();
		if (autoCommand != nullptr) autoCommand->Cancel();
		encoder_left->Reset();
		encoder_right->Reset();
		odom->Reset();
		MoveClawsIn();
		autoCommand = auto_chooser.GetSelected();
		autoCommand->Start();
	}
	void AutonomousPeriodic() override
	{

		//TestPeriodic();
		Scheduler::GetInstance()->Run();
	}

	void TeleopInit() override
	{
		encoder_left->Reset();
		encoder_right->Reset();
		odom->Reset();
#ifdef USE_ROS
		getNodeHandle().loginfo("Teleop Begin");
#endif
#ifdef USE_VEL_TELEOP
		SetWrenchVelocity();
#else
		SetWrenchEffort();
#endif
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
		//drive->Drive(rotation/s, x/s);
		drive->ArcadeDrive(rotation, x/s);
#endif
		if (stickLeft->GetRawButton(4)) climber->Set(-1);
		else if (stickLeft->GetRawButton(5)) climber->Set(-0.25);
		else climber->Set(0);
		if (stickMiddle->GetRawButton(3)) MoveClawsOut();
		else MoveClawsIn();
}
	void DisabledInit() override
	{
#ifdef USE_ROS
		getNodeHandle().loginfo("Disabled");
#endif
		drive_ctrl->Disable();

	}
	void DisabledPeriodic() override
	{

	}
	void TestInit() override
	{
#ifdef USE_ROS
		getNodeHandle().loginfo("Test Begin");
#endif

		RobotMap::P = SmartDashboard::GetNumber("P", RobotMap::P);
		RobotMap::D = SmartDashboard::GetNumber("D", RobotMap::D);
		left_controller->SetPID(RobotMap::P, RobotMap::I, RobotMap::D);
		right_controller->SetPID(RobotMap::P, RobotMap::I, RobotMap::D);
		printf("P = %f, D= %f\n", P, D);
		/* Tunning velocity PID
		 * 1) Drive robot at max effort, record velocity
		 * 2) Set velocity as feed forward to left + right controller
		 * 3) kD is multiple of current velocity error
		 * 4) kP is multiple of integrated velocity error
		 */
		SetWrenchVelocity();
		geometry_msgs::Twist twist_msg;
		//twist_msg.linear.x = 1.5;
		twist_msg.angular.z = 3.141592654;
		drive_ctrl->Set(twist_msg);

	}
	void TestPeriodic() override
	{

	}
	void RobotPeriodic () override
	{
		if (encoder_left->GetRate() > max_vel) max_vel =encoder_left->GetRate();
//		std::cout << "Cam: " << cam.IsConnected() << std::endl;
#ifdef USE_ROS

#else
		odom->update();
		SmartDashboard::PutNumber("Odom Theta", odom->GetThetaDeg());
		SmartDashboard::PutNumber("Odom X", odom->GetX());
		SmartDashboard::PutNumber("Odom Y", odom->GetY());
		SmartDashboard::PutNumber("Encoder Left", encoder_left->GetDistance());
		SmartDashboard::PutNumber("Encoder Right", encoder_right->GetDistance());
		SmartDashboard::PutNumber("Encoder Left Rate", encoder_left->GetRate());
		SmartDashboard::PutNumber("Encoder Right Rate", encoder_right->GetRate());
		SmartDashboard::PutNumber("Left Goal",		left_controller->GetSetpoint());
		SmartDashboard::PutNumber("Right Goal", right_controller->GetSetpoint());
		SmartDashboard::PutNumber("Max", max_vel);
		//printf("Encoder Left = %f\n", encoder_left->GetDistance());
		//printf("Encoder Right = %f\n\n", encoder_right->GetDistance());
#endif
	}
};

START_ROBOT_CLASS(Robot)
