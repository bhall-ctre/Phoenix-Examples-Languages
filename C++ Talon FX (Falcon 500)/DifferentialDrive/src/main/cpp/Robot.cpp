/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */
/**
 * Enable robot and slowly drive forward.
 * [1] If DS reports errors, adjust CAN IDs and firmware update.
 * [2] If motors are spinning incorrectly, first check gamepad.
 * [3] If motors are still spinning incorrectly, correct motor inverts.
 * [4] Now that motors are driving correctly, check sensor phase.  If sensor is out of phase, adjust sensor phase.
 */
#include <iostream>
#include <string>

#include "frc/WPILib.h"
#include "frc/kinematics/DifferentialDriveOdometry.h"
#include "frc/simulation/DifferentialDrivetrainSim.h"
#include "frc/smartdashboard/Field2d.h"
#include "units/units.h"
#include "ctre/Phoenix.h"

using namespace frc;

class Robot: public TimedRobot {
public:
	/* ------ [1] Update CAN Device IDs and switch to WPI_VictorSPX where necessary ------*/
	WPI_TalonFX _rghtFront{1};
	WPI_TalonFX _rghtFollower{3};
	WPI_TalonFX _leftFront{2};
	WPI_TalonFX _leftFollower{4};

	WPI_PigeonIMU _pigeon{0};

	DifferentialDrive _diffDrive{_leftFront, _rghtFront};

	Joystick _joystick{0};

	void TeleopPeriodic() {

		std::stringstream work;

		/* get gamepad stick values */
		double forw = -1 * _joystick.GetRawAxis(1); /* positive is forward */
		double turn = +1 * _joystick.GetRawAxis(2); /* positive is right */

		/* deadband gamepad 10%*/
		if (fabs(forw) < 0.10)
			forw = 0;
		if (fabs(turn) < 0.10)
			turn = 0;

		/* drive robot */
		_diffDrive.ArcadeDrive(forw, turn, false);

		/* -------- [2] Make sure Gamepad Forward is positive for FORWARD, and GZ is positive for RIGHT */
		work << " GF:" << forw << " GT:" << turn;

		/* get sensor values */
		//double leftPos = _leftFront.GetSelectedSensorPosition(0);
		//double rghtPos = _rghtFront.GetSelectedSensorPosition(0);
		double leftVelUnitsPer100ms = _leftFront.GetSelectedSensorVelocity(0);
		double rghtVelUnitsPer100ms = _rghtFront.GetSelectedSensorVelocity(0);

		work << " L:" << leftVelUnitsPer100ms << " R:" << rghtVelUnitsPer100ms;

		/* print to console */
		std::cout << work.str() << std::endl;
	}

	void RobotInit() {
		/* factory default values */
		_rghtFront.ConfigFactoryDefault();
		_rghtFollower.ConfigFactoryDefault();
		_leftFront.ConfigFactoryDefault();
		_leftFollower.ConfigFactoryDefault();

		/* set up followers */
		_rghtFollower.Follow(_rghtFront);
		_leftFollower.Follow(_leftFront);

		/* [3] flip values so robot moves forward when stick-forward/LEDs-green */
		_rghtFront.SetInverted(TalonFXInvertType::Clockwise);
		_rghtFollower.SetInverted(TalonFXInvertType::FollowMaster);
		_leftFront.SetInverted(TalonFXInvertType::CounterClockwise);
		_leftFollower.SetInverted(TalonFXInvertType::FollowMaster);

		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
		// _rghtFront.SetSensorPhase(true);
		// _leftFront.SetSensorPhase(true);

		/*
		* WPI drivetrain classes defaultly assume left and right are opposite. call
		* this so we can apply + to both sides when moving forward. DO NOT CHANGE
		*/
		_diffDrive.SetRightSideInverted(false);

		frc::SmartDashboard::PutData(wpi::StringRef("Field"), &_field);
	}

	void RobotPeriodic() {
		// This will get the simulated sensor readings that we set
		// in the previous article while in simulation, but will use
		// real values on the robot itself.
		_odometry.Update(_pigeon.GetRotation2d(),
						units::meter_t{NativeUnitsToDistanceMeters(_leftFront.GetSelectedSensorPosition())},
						units::meter_t{NativeUnitsToDistanceMeters(_rghtFront.GetSelectedSensorPosition())});
		_field.SetRobotPose(_odometry.GetPose());
	}

	// Object for simulated inputs into Talon.
	TalonFXSimCollection _leftFrontSim = _leftFront.GetSimCollection();
	TalonFXSimCollection _rghtFrontSim = _rghtFront.GetSimCollection();
	// Object for simulated inputs into Pigeon.
	PigeonIMUSimCollection _pigeonSim = _pigeon.GetSimCollection();
	
	//These numbers are an example AndyMark Drivetrain with some additional weight.  This is a fairly light robot.
	//Note you can utilize results from robot characterization instead of theoretical numbers.
	//https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html#introduction-to-robot-characterization
	const int kCountsPerRev = 2048;  //Encoder counts per revolution of the motor shaft.
	const double kSensorGearRatio = 1; //Gear ratio is the ratio between the *encoder* and the wheels.  On the AndyMark drivetrain, encoders mount 1:1 with the gearbox shaft.
	const double kGearRatio = 10.71; //Switch kSensorGearRatio to this gear ratio if encoder is on the motor instead of on the gearbox.
	const units::inch_t kWheelRadiusInches = 3_in;
	const int k100msPerSecond = 10;

	//Simulation model of the drivetrain
	frc::sim::DifferentialDrivetrainSim _driveSim{
		frc::DCMotor::Falcon500(2),  //2 Falcon 500s on each side of the drivetrain.
		kGearRatio,                  //Standard AndyMark Gearing reduction.
		2.1_kg_sq_m,                 //MOI of 2.1 kg m^2 (from CAD model).
		26.5_kg,                     //Mass of the robot is 26.5 kg.
		kWheelRadiusInches,          //Robot uses 3" radius (6" diameter) wheels.
		0.546_m,                     //Distance between wheels is _ meters.
		
		// The standard deviations for measurement noise:
		// x and y:          0.001 m
		// heading:          0.001 rad
		// l and r velocity: 0.1   m/s
		// l and r position: 0.005 m
		//VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005) //Uncomment this line to add measurement noise.
	};

	frc::Field2d _field;
	// Creating my odometry object. Here,
	// our starting pose is 5 meters along the long end of the field and in the
	// center of the field along the short end, facing forward.
	frc::DifferentialDriveOdometry _odometry{_pigeon.GetRotation2d()};

	void SimulationPeriodic() {
		// Set the inputs to the system. Note that we need to use
		// the output voltage, NOT the percent output.
		_driveSim.SetInputs(units::volt_t{_leftFrontSim.GetMotorOutputLeadVoltage()},
							units::volt_t{-_rghtFrontSim.GetMotorOutputLeadVoltage()}); //Right side is inverted, so forward is negative voltage

		// Advance the model by 20 ms. Note that if you are running this
		// subsystem in a separate thread or have changed the nominal timestep
		// of TimedRobot, this value needs to match it.
		_driveSim.Update(0.02_s);

		// Update all of our sensors.
		_leftFrontSim.SetIntegratedSensorRawPosition(
						DistanceToNativeUnits(
						_driveSim.GetLeftPosition().to<double>()));
		_leftFrontSim.SetIntegratedSensorVelocity(
						VelocityToNativeUnits(
						_driveSim.GetLeftVelocity().to<double>()));
		_rghtFrontSim.SetIntegratedSensorRawPosition(
						DistanceToNativeUnits(
						-_driveSim.GetRightPosition().to<double>()));
		_rghtFrontSim.SetIntegratedSensorVelocity(
						VelocityToNativeUnits(
						-_driveSim.GetRightVelocity().to<double>()));
		_pigeonSim.SetRawHeading(_driveSim.GetHeading().Degrees().to<double>());

		//Update other inputs to Talons
		_leftFrontSim.SetBusVoltage(RobotController::GetInputVoltage());
		_rghtFrontSim.SetBusVoltage(RobotController::GetInputVoltage());
	}

private:
	int DistanceToNativeUnits(double positionMeters){
		double wheelRotations = positionMeters/(2 * wpi::math::pi * kWheelRadiusInches.convert<units::meter>().to<double>());
		double motorRotations = wheelRotations * kSensorGearRatio;
		int sensorCounts = (int)(motorRotations * kCountsPerRev);
		return sensorCounts;
	}

	int VelocityToNativeUnits(double velocityMetersPerSecond){
		double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * wpi::math::pi * kWheelRadiusInches.convert<units::meter>().to<double>());
		double motorRotationsPerSecond = wheelRotationsPerSecond * kSensorGearRatio;
		double motorRotationsPer100ms = motorRotationsPerSecond / k100msPerSecond;
		int sensorCountsPer100ms = (int)(motorRotationsPer100ms * kCountsPerRev);
		return sensorCountsPer100ms;
	}

	double NativeUnitsToDistanceMeters(double sensorCounts){
		double motorRotations = (double)sensorCounts / kCountsPerRev;
		double wheelRotations = motorRotations / kSensorGearRatio;
		double positionMeters = wheelRotations * (2 * wpi::math::pi * kWheelRadiusInches.convert<units::meter>().to<double>());
		return positionMeters;
	}
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif