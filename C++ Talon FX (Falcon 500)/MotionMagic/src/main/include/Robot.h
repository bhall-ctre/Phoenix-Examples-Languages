#pragma once

#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include "ctre/Phoenix.h"
#include "Instrum.h"

class Robot : public frc::TimedRobot
{
public:
	void SimulationInit() override;
	void SimulationPeriodic() override;

	void RobotInit() override;

	void AutonomousInit() override;
	void AutonomousPeriodic() override;

	void TeleopInit() override;
	void TeleopPeriodic() override;

	void TestInit() override;
	void TestPeriodic() override;

private:
	WPI_TalonFX *_talon;
	frc::Joystick *_joy;

	int _smoothing;
};
