#include <RobotMap.h>

using namespace std;
shared_ptr<frc::Joystick> RobotMap::stickLeft = NULL;
shared_ptr<frc::Joystick> RobotMap::stickMiddle = NULL;
shared_ptr<frc::Joystick> RobotMap::stickRight = NULL;
shared_ptr<frc::RobotDrive> RobotMap::drive = NULL;
shared_ptr<TwoMotorOutput> RobotMap::drive_left = NULL;
shared_ptr<TwoMotorOutput> RobotMap::drive_right = NULL;
shared_ptr<frc::PIDController> RobotMap::left_controller = NULL;
shared_ptr<frc::PIDController> RobotMap::right_controller = NULL;
shared_ptr<DiffDriveController> RobotMap::drive_ctrl = NULL;
shared_ptr<EncoderOdometry> RobotMap::odom = NULL;
double RobotMap::P = 0.1;
double RobotMap::I = 0.0;
double RobotMap::D = 0.325;
shared_ptr<BaseServo> RobotMap::servo_left = NULL;
shared_ptr<BaseServo> RobotMap::servo_right = NULL;
std::shared_ptr<frc::Encoder> RobotMap::encoder_left = NULL;
std::shared_ptr<frc::Encoder> RobotMap::encoder_right = NULL;
std::shared_ptr<TwoMotorOutput> RobotMap::climber = NULL;


void RobotMap::init(ros::NodeHandle& nh)
{
	stickLeft.reset(new frc::Joystick(0));
	stickMiddle.reset(new frc::Joystick(1));
	stickRight.reset(new frc::Joystick(2));

	auto BL = shared_ptr<frc::SpeedController>(new frc::VictorSP(0));
	auto BR = shared_ptr<frc::SpeedController>(new frc::VictorSP(2));
	auto FL = shared_ptr<frc::SpeedController>(new frc::VictorSP(1));
	auto FR = shared_ptr<frc::SpeedController>(new frc::VictorSP(3));
	FR->SetInverted(true);
	BR->SetInverted(true);
	drive_left.reset(new TwoMotorOutput(BL, FL));
	drive_right.reset(new TwoMotorOutput(BR, FR));
	drive.reset(new RobotDrive(*FL, *BL, *FR, *BR));
	left_controller.reset(new PIDController(P, I, D, DRIVE_CONTROLLER_KF, encoder_left.get(), drive_left.get()));
	right_controller.reset(new PIDController(P, I, D, DRIVE_CONTROLLER_KF, encoder_right.get(), drive_right.get()));
	odom.reset(new EncoderOdometry(nh, "/odom", encoder_left, encoder_right, WHEEL_SEPERATION_METERS));
	drive_ctrl.reset(new DiffDriveController(nh, "/cmd_vel", left_controller, right_controller, WHEEL_SEPERATION_METERS));

	auto climberOne = shared_ptr<frc::SpeedController>(new frc::VictorSP(4));
	auto climberTwo = shared_ptr<frc::SpeedController>(new frc::VictorSP(5));
	climber.reset(new TwoMotorOutput(climberOne, climberTwo));

	servo_left.reset(new ServoHS805BB(6));
	servo_right.reset(new ServoHS805BB(7));
	encoder_left.reset(new Encoder(7 ,8, false, frc::Encoder::k4X));
	encoder_right.reset(new Encoder(5, 6,false, frc::Encoder::k4X));
}
void RobotMap::MoveClawsOut(){
	servo_left->SetAngle(0);
	servo_right->SetAngle(95);
}
void RobotMap::MoveClawsIn(){
	servo_left->SetAngle(95);
	servo_right->SetAngle(0);
}
