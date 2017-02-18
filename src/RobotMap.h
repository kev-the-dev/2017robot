#pragma once

#include <memory>

#include <rosfrc/RosRobot.h>
#include <RobotDrive.h>
#include <VictorSP.h>
#include <Encoder.h>
#include <Servo.h>
#include <Joystick.h>
#include "BaseServo.h"
#include "Talon.h"
#include "Victor.h"
#include "EncoderOdometry.hpp"
#include "DiffDriveController.hpp"
#include "TwoMotorOutput.hpp"

class RobotMap {
public:
	static constexpr double WHEEL_RADIUS_METERS = 0.0762;
	static constexpr double CYCLES_PER_REVOLUTION = 360.0;
	static constexpr double WHEEL_SEPERATION_METERS = 0.6096;
	static constexpr double ENCODER_REVOLUTIONS_PER_OUTPUT_REVOLUTION = 11.25;
	static constexpr double MAX_LINEAR_VELOCITY_METERS_PER_SECOND = 5.4;
	static constexpr double MAX_ANGULAR_VELOCITY = 6.0;

	static constexpr double WHEEL_CIRCUMFERENCE_METERS = 2.0 * PI * WHEEL_RADIUS_METERS;
	static constexpr double DISTANCE_PER_CYCLE_METERS = (WHEEL_CIRCUMFERENCE_METERS / CYCLES_PER_REVOLUTION) / ENCODER_REVOLUTIONS_PER_OUTPUT_REVOLUTION;
	static constexpr double DRIVE_CONTROLLER_KF = 1.0 / MAX_LINEAR_VELOCITY_METERS_PER_SECOND;

	static void init(ros::NodeHandle &nh);

	// HID
	static std::shared_ptr<frc::Joystick> stickLeft;
	static std::shared_ptr<frc::Joystick> stickRight;
	static std::shared_ptr<frc::Joystick> stickMiddle;

	// Drive
	static double P, I, D;
	static std::shared_ptr<TwoMotorOutput> drive_left;
	static std::shared_ptr<TwoMotorOutput> drive_right;
	static std::shared_ptr<frc::PIDController> left_controller;
	static std::shared_ptr<frc::PIDController> right_controller;
	static std::shared_ptr<DiffDriveController> drive_ctrl;
	static std::shared_ptr<EncoderOdometry> odom;
	static std::shared_ptr<RobotDrive> drive;
	static std::shared_ptr<frc::Encoder> encoder_left;
	static std::shared_ptr<frc::Encoder> encoder_right;


	// Climber
	static std::shared_ptr<TwoMotorOutput> climber;

	// Gear mechanism
	static std::shared_ptr<BaseServo> servo_left;
	static std::shared_ptr<BaseServo> servo_right;

	void MoveClawsOut();
	void MoveClawsIn();

};

