
#pragma once

#include <memory>
#include <string>

#include "SafePWM.h"
#include "SpeedController.h"

class BaseServo : public SafePWM {
 protected:
  explicit BaseServo(int channel,
		  double minAngle,
		  double maxAngle,
		  double minPWM,
		  double maxPWM);
 public:
  virtual ~BaseServo();
  void Set(double value);
  void SetOffline();
  double Get() const;
  void SetAngle(double angle);
  double GetAngle() const;
  double GetMaxAngle() { return kMaxServoAngle; }
  double GetMinAngle() { return kMinServoAngle; }

  void ValueChanged(ITable* source, llvm::StringRef key,
                    std::shared_ptr<nt::Value> value, bool isNew) override;
  void UpdateTable() override;
  void StartLiveWindowMode() override;
  void StopLiveWindowMode() override;
  std::string GetSmartDashboardType() const override;
  void InitTable(std::shared_ptr<ITable> subTable) override;
  std::shared_ptr<ITable> GetTable() const override;

  std::shared_ptr<ITable> m_table;

 private:
  double kMinServoAngle;
  double kMaxServoAngle;
  double kMinServoPWM;
  double kMaxServoPWM;
  double GetServoAngleRange() const { return kMaxServoAngle - kMinServoAngle; }
};

class ServoHS805BB: public BaseServo
{
public:
	ServoHS805BB(int channel) :
		BaseServo(channel, 0, 199.5, 0.6, 2.4)
	{

	}
};
