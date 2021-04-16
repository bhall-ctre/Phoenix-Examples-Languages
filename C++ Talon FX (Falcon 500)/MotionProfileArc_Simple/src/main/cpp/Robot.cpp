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
#include "Robot.h"
#include "MotionProfile.h"
#include "Instrum.h"

void Robot::RobotInit() 
{
    /* Initialize buffer with motion profile */
    InitBuffer(kMotionProfile, kMotionProfileSz, 0.25); //Do a quarter (0.25) rotation to the left
    _state = 0;

    _rightMaster.ConfigAllSettings(_masterConfig);
    _leftMaster.ConfigAllSettings(_followConfig);

    /*
     * Talon FX does not need sensor phase set for its integrated sensor
     * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
     * and the user calls getSelectedSensor* to get the sensor's position/velocity.
     * 
     * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
     */
    // _rightMaster.SetSensorPhase(true);
    // _leftMaster.SetSensorPhase(false);

    _rightMaster.SetInverted(TalonFXInvertType::Clockwise);
    _leftMaster.SetInverted(TalonFXInvertType::CounterClockwise);

    _rightMaster.SetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 20); //Telemetry using Phoenix Tuner

	frc::SmartDashboard::PutData(wpi::StringRef("Field"), &_field);
}

void Robot::RobotPeriodic() {
    // This will get the simulated sensor readings that we set
    // in the previous article while in simulation, but will use
    // real values on the robot itself.
    _odometry.Update(_pidgey.GetRotation2d(),
                    units::meter_t{NativeUnitsToDistanceMeters(_leftMaster.GetSelectedSensorPosition())},
                    units::meter_t{NativeUnitsToDistanceMeters(_rightMaster.GetSelectedSensorPosition())});
    _field.SetRobotPose(_odometry.GetPose());
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() 
{
    /* get joystick button and stick */
    bool bPrintValues = _joystick.GetRawButton(2);
    bool bFireMp = _joystick.GetRawButton(1);
    double axis = -_joystick.GetRawAxis(1);
    double turn = _joystick.GetRawAxis(2);

    /* if button is up, just drive the motor in PercentOutput */
    if (bFireMp == false) {
        _state = 0;
    }

    switch (_state) {
        /* drive master talon normally */
        case 0:
            _rightMaster.Set(ControlMode::PercentOutput, axis, DemandType_ArbitraryFeedForward, -turn);
            _leftMaster.Set(ControlMode::PercentOutput, axis, DemandType_ArbitraryFeedForward, turn);
            if (bFireMp == true) {
                /* go to MP logic */
                _state = 1;
            }
            break;

        /* fire the MP, and stop calling set() since that will cancel the MP */
        case 1:
            _rightMaster.GetSensorCollection().SetIntegratedSensorPosition(0);
            _leftMaster.GetSensorCollection().SetIntegratedSensorPosition(0);
            _pidgey.SetYaw(0);
            /* wait for 10 points to buffer in firmware, then transition to MP */
            _leftMaster.Follow(_rightMaster, FollowerType_AuxOutput1);
            _rightMaster.StartMotionProfile(_bufferedStream, 10, ControlMode::MotionProfileArc);
            _state = 2;
            Instrum::PrintLine("MP started");
            break;

        /* wait for MP to finish */
        case 2:
            if (_rightMaster.IsMotionProfileFinished()) {
                Instrum::PrintLine("MP finished");
                _state = 3;
            }
            break;

        /* MP is finished, nothing to do */
        case 3:
            break;
    }

    /* print MP values */
    Instrum::Loop(bPrintValues, _rightMaster);
}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::InitBuffer(const double profile[][3], int totalCnt, double rotations)
{
    bool forward = true; // set to false to drive in opposite direction of profile (not really needed
                         // since you can use negative numbers in profile).

    TrajectoryPoint point; // temp for for loop, since unused params are initialized
                           // automatically, you can alloc just one

    /* clear the buffer, in case it was used elsewhere */
    _bufferedStream.Clear();

    double turnAmount = rotations * 8192.0; //8192 units per rotation for a pigeon


    /* Insert every point into buffer, no limit on size */
    for (int i = 0; i < totalCnt; ++i) {

        double direction = forward ? +1 : -1;
        double positionRot = profile[i][0];
        double velocityRPM = profile[i][1];
        int durationMilliseconds = (int) profile[i][2];

        /* for each point, fill our structure and pass it to API */
        point.timeDur = durationMilliseconds;
        point.position = direction * positionRot * 2048; // Convert Revolutions to
                                                         // Units
        point.velocity = direction * velocityRPM * 2048 / 600.0; // Convert RPM to
                                                                 // Units/100ms
        
        /** 
         * Here is where you specify the heading of the robot at each point. 
         * In this example we're linearly interpolating creating a segment of a circle to follow
         */
        point.auxiliaryPos = turnAmount * ((double)i / (double)totalCnt); //Linearly interpolate the turn amount to do a circle
        point.auxiliaryVel = 0;


        point.profileSlotSelect0 = 0; /* which set of gains would you like to use [0,3]? */
        point.profileSlotSelect1 = 1; /* which set of gains would you like to use [0,3]? */
        point.zeroPos = (i == 0); /* set this to true on the first point */
        point.isLastPoint = ((i + 1) == totalCnt); /* set this to true on the last point */
        point.arbFeedFwd = 0; /* you can add a constant offset to add to PID[0] output here */

        point.useAuxPID = true; /* Using auxiliary PID */
        _bufferedStream.Write(point);
    }
}

void Robot::SimulationPeriodic() {
    // Set the inputs to the system. Note that we need to use
    // the output voltage, NOT the percent output.
    _driveSim.SetInputs(units::volt_t{_leftMasterSim.GetMotorOutputLeadVoltage()},
                        units::volt_t{-_rightMasterSim.GetMotorOutputLeadVoltage()}); //Right side is inverted, so forward is negative voltage

    // Advance the model by 20 ms. Note that if you are running this
    // subsystem in a separate thread or have changed the nominal timestep
    // of TimedRobot, this value needs to match it.
    _driveSim.Update(0.02_s);

    // Update all of our sensors.
    _leftMasterSim.SetIntegratedSensorRawPosition(
                    DistanceToNativeUnits(
                    _driveSim.GetLeftPosition().to<double>()));
    _leftMasterSim.SetIntegratedSensorVelocity(
                    VelocityToNativeUnits(
                    _driveSim.GetLeftVelocity().to<double>()));
    _rightMasterSim.SetIntegratedSensorRawPosition(
                    DistanceToNativeUnits(
                    -_driveSim.GetRightPosition().to<double>()));
    _rightMasterSim.SetIntegratedSensorVelocity(
                    VelocityToNativeUnits(
                    -_driveSim.GetRightVelocity().to<double>()));
    _pidgeySim.SetRawHeading(_driveSim.GetHeading().Degrees().to<double>());

    //Update other inputs to Talons
    _leftMasterSim.SetBusVoltage(frc::RobotController::GetInputVoltage());
    _rightMasterSim.SetBusVoltage(frc::RobotController::GetInputVoltage());
}

int Robot::DistanceToNativeUnits(double positionMeters){
    double wheelRotations = positionMeters/(2 * wpi::math::pi * kWheelRadiusInches.convert<units::meter>().to<double>());
    double motorRotations = wheelRotations * kSensorGearRatio;
    int sensorCounts = (int)(motorRotations * kCountsPerRev);
    return sensorCounts;
}

int Robot::VelocityToNativeUnits(double velocityMetersPerSecond){
    double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * wpi::math::pi * kWheelRadiusInches.convert<units::meter>().to<double>());
    double motorRotationsPerSecond = wheelRotationsPerSecond * kSensorGearRatio;
    double motorRotationsPer100ms = motorRotationsPerSecond / k100msPerSecond;
    int sensorCountsPer100ms = (int)(motorRotationsPer100ms * kCountsPerRev);
    return sensorCountsPer100ms;
}

double Robot::NativeUnitsToDistanceMeters(double sensorCounts){
    double motorRotations = (double)sensorCounts / kCountsPerRev;
    double wheelRotations = motorRotations / kSensorGearRatio;
    double positionMeters = wheelRotations * (2 * wpi::math::pi * kWheelRadiusInches.convert<units::meter>().to<double>());
    return positionMeters;
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
