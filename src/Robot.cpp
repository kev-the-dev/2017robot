
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

#include <thread>
#include <chrono>

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


class Robot: public rosfrc::RosRobot, RobotMap {
public:
	SendableChooser<std::shared_ptr<Command>> auto_chooser;
	Robot():
			RosRobot(KANGAROO_IP)
	{
		RobotMap::init(getNodeHandle());
		AddUpdater(odom.get());
		AddEncoder("/encoder_left", encoder_left);
		AddEncoder("/encoder_right", encoder_right);

		auto_chooser.AddDefault("Do Nothing",std::shared_ptr<Command>(new DoNothing()));
		auto_chooser.AddObject ("Move Forward", std::shared_ptr<Command>(new DriveForwardAuto()));
		auto_chooser.AddObject ("Center Gear", std::shared_ptr<Command>(new CenterGear()));
		SmartDashboard::PutData("Auto Program", &auto_chooser);
	}
	void AutonomousInit() override
	{
		odom->Reset();
		MoveClawsIn();
		getNodeHandle().loginfo("Auto Begin");
		std::shared_ptr<Command> autoCommand = auto_chooser.GetSelected();
		autoCommand->Start();
	}
	void AutonomousPeriodic() override
	{
		Scheduler::GetInstance()->Run();
	}

	void TeleopInit() override
	{
		getNodeHandle().loginfo("Teleop Begin");
#ifdef USE_VEL_TELEOP
		drive_ctrl->Enable();
#else
		drive_ctrl->Disable();
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
		drive->ArcadeDrive(rotation/s, x/s);
#endif
		if (stickLeft->GetRawButton(4)) climber->Set(-1);
		else if (stickLeft->GetRawButton(5)) climber->Set(-0.25);
		else climber->Set(0);
		if (stickMiddle->GetRawButton(3)) MoveClawsOut();
		else MoveClawsIn();
}
	void DisabledInit() override
	{
		drive_ctrl->Disable();
		getNodeHandle().loginfo("Disabled");

	}
	void DisabledPeriodic() override
	{

	}
	void TestInit() override
	{
		getNodeHandle().loginfo("Test Begin");

		/* Tunning velocity PID
		 * 1) Drive robot at max effort, record velocity
		 * 2) Set velocity as feed forward to left + right controller
		 * 3) kD is multiple of current velocity error
		 * 4) kP is multiple of integrated velocity error
		 */
		drive_ctrl->Enable();
		geometry_msgs::Twist twist_msg;
		twist_msg.linear.x = 1.5;
		drive_ctrl->Set(twist_msg);

	}
	void TestPeriodic() override
	{

	}
	void RobotPeriodic () override
	{

	}
};

START_ROBOT_CLASS(Robot)
