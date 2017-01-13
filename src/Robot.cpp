
// Ros stuff
#include <rosfrc/RosRobot.h>
#include <std_msgs/String.h>
#include <RobotDrive.h>
#include <Talon.h>
#include <memory>

class Robot: public RosRobot {
public:
	ros::Publisher chatter;
	std::shared_ptr<frc::Joystick> stickLeft;
	std::shared_ptr<frc::Talon> BL;
	std::shared_ptr<frc::Talon> BR;
	std::shared_ptr<frc::Talon> FL;
	std::shared_ptr<frc::Talon> FR;
	frc::RobotDrive drive;
	std_msgs::String str_msg;
	Robot():
			RosRobot("10.41.18.94"),
			chatter("chatter", &str_msg),
			stickLeft(new frc::Joystick(0)),
			BL(new Talon(3)),
			BR(new Talon(2)),
			FL(new Talon(0)),
			FR(new Talon(1)),
			drive(*FL, *BL, *FR, *BR)
	{
		FR->SetInverted(true);
		BR->SetInverted(true);
		frc::LiveWindow::GetInstance()->AddActuator(std::string("DRIVING"),std::string("BL"), *BL);
		frc::LiveWindow::GetInstance()->AddActuator(std::string("DRIVING"),std::string("BR"), *BR);
		frc::LiveWindow::GetInstance()->AddActuator(std::string("DRIVING"),std::string("FL"), *FL);
		frc::LiveWindow::GetInstance()->AddActuator(std::string("DRIVING"),std::string("FR"), *FR);
		AddJoystick("/joysticks/left", stickLeft);
		AddSpeedController("/speed_controllers/FL", FL);
		AddSpeedController("/speed_controllers/FR", FR);
		AddSpeedController("/speed_controllers/BL", BL);
		AddSpeedController("/speed_controllers/BR", BR);
		getRosNodeHandle().advertise(chatter);
	}
	void RobotPeriodic () override
	{
		str_msg.data = "HERRO";
		chatter.publish(&str_msg);
	}
};

START_ROBOT_CLASS(Robot)
