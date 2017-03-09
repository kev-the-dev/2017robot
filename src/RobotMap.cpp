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
double RobotMap::P = 0.0;
double RobotMap::I = 0.0;
double RobotMap::D = 0.3;
shared_ptr<BaseServo> RobotMap::servo_left = NULL;
shared_ptr<BaseServo> RobotMap::servo_right = NULL;
std::shared_ptr<frc::Encoder> RobotMap::encoder_left = NULL;
std::shared_ptr<frc::Encoder> RobotMap::encoder_right = NULL;
std::shared_ptr<TwoMotorOutput> RobotMap::climber = NULL;


#ifdef USE_ROS
void RobotMap::init(ros::NodeHandle& nh)
#else
void RobotMap::init()
#endif
{
	stickLeft.reset(new frc::Joystick(0));
	stickMiddle.reset(new frc::Joystick(1));
	stickRight.reset(new frc::Joystick(2));

	encoder_left.reset(new Encoder(7 ,8, true, frc::Encoder::k4X));
	encoder_right.reset(new Encoder(6, 5,true, frc::Encoder::k4X));
	encoder_left->SetDistancePerPulse(DISTANCE_PER_CYCLE_METERS);
	encoder_right->SetDistancePerPulse(DISTANCE_PER_CYCLE_METERS);
	encoder_left->SetPIDSourceType(PIDSourceType::kRate);
	encoder_right->SetPIDSourceType(PIDSourceType::kRate);
	auto BL = shared_ptr<frc::SpeedController>(new frc::VictorSP(0));
	auto BR = shared_ptr<frc::SpeedController>(new frc::VictorSP(2));
	auto FL = shared_ptr<frc::SpeedController>(new frc::VictorSP(1));
	auto FR = shared_ptr<frc::SpeedController>(new frc::VictorSP(3));
	drive_left.reset(new TwoMotorOutput(BL, FL));
	drive_right.reset(new TwoMotorOutput(BR, FR));
	drive_right->SetInverted(true);
	drive.reset(new RobotDrive(drive_left, drive_right));
	drive->SetExpiration(3);
	left_controller.reset(new PIDController(P, I, D, DRIVE_CONTROLLER_KF, encoder_left.get(), drive_left.get()));
	right_controller.reset(new PIDController(P, I, D, DRIVE_CONTROLLER_KF, encoder_right.get(), drive_right.get()));
	left_controller->SetPIDSourceType(PIDSourceType::kRate);
	right_controller->SetPIDSourceType(PIDSourceType::kRate);
#ifdef USE_ROS
	odom.reset(new EncoderOdometry(nh, "/odom", encoder_left, encoder_right, WHEEL_SEPERATION_METERS));
	drive_ctrl.reset(new DiffDriveController(nh, "/cmd_vel", left_controller, right_controller, WHEEL_SEPERATION_METERS));
#else
	odom.reset(new EncoderOdometry("/odom", encoder_left, encoder_right, WHEEL_SEPERATION_METERS));
	drive_ctrl.reset(new DiffDriveController("/cmd_vel", left_controller, right_controller, WHEEL_SEPERATION_METERS));
#endif

	auto climberOne = shared_ptr<frc::SpeedController>(new frc::VictorSP(4));
	auto climberTwo = shared_ptr<frc::SpeedController>(new frc::VictorSP(5));
	climber.reset(new TwoMotorOutput(climberOne, climberTwo));

	servo_left.reset(new ServoHS805BB(6));
	servo_right.reset(new ServoHS805BB(7));
}
void RobotMap::MoveClawsOut(){
	servo_left->SetAngle(95);
	servo_right->SetAngle(95);
}
void RobotMap::MoveClawsIn(){
	servo_left->SetAngle(195);
	servo_right->SetAngle(0);
}
void RobotMap::SetWrenchEffort()
{
	drive_ctrl->Disable();
	drive->SetSafetyEnabled(true);
	drive->Drive(0.0, 0.0);
}
void RobotMap::SetWrenchVelocity()
{
	drive->SetSafetyEnabled(false);
	drive_ctrl->Enable();
	drive_ctrl->Set(0.0, 0.0);
}
