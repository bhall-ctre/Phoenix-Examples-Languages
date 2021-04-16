
#pragma once

#include <frc/TimedRobot.h>
#include <frc/WPILib.h>
#include <ctre/Phoenix.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

private:
  WPI_CANCoder _CANCoder{0};
  frc::Joystick _joy{0};
};
