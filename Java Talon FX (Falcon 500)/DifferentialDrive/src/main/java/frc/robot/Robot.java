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
 * [2] If motors are spinning incorrectly, first check gamepad (hold down btn1)
 * [3] If motors are still spinning incorrectly, correct motor inverts.
 * [4] Now that motors are driving correctly, check sensor phase.  If sensor is out of phase, adjust sensor phase.
 * [4] Is only necessary if you have sensors.
 */
package frc.robot;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
    /*
     * --- [1] Update CAN Device IDs ------
     */
    WPI_TalonFX _rghtFront = new WPI_TalonFX(1);
    WPI_TalonFX _rghtFollower = new WPI_TalonFX(10);
    WPI_TalonFX _leftFront = new WPI_TalonFX(2);
    WPI_TalonFX _leftFollower = new WPI_TalonFX(20);

    WPI_PigeonIMU _pidgey = new WPI_PigeonIMU(0);

    DifferentialDrive _diffDrive = new DifferentialDrive(_leftFront, _rghtFront);

    Joystick _joystick = new Joystick(0);

    Faults _faults_L = new Faults();
    Faults _faults_R = new Faults();

    DrivebaseSimFX _driveSim = new DrivebaseSimFX(_leftFront, _rghtFront, _pidgey);

    @Override
    public void simulationPeriodic() {
        _driveSim.run();
    }

    @Override
    public void teleopPeriodic() {

        String work = "";

        /* get gamepad stick values */
        double forw = -1 * _joystick.getRawAxis(1); /* positive is forward */
        double turn = +1 * _joystick.getRawAxis(2); /* positive is right */
        boolean btn1 = _joystick.getRawButton(1); /* is button is down, print joystick values */

        /* deadband gamepad 10% */
        if (Math.abs(forw) < 0.10) {
            forw = 0;
        }
        if (Math.abs(turn) < 0.10) {
            turn = 0;
        }

        /* drive robot */
        _diffDrive.arcadeDrive(forw, turn);

        /*
         * [2] Make sure Gamepad Forward is positive for FORWARD, and GZ is positive for
         * RIGHT
         */
        work += " GF:" + forw + " GT:" + turn;

        /* get sensor values */
        // double leftPos = _leftFront.GetSelectedSensorPosition(0);
        // double rghtPos = _rghtFront.GetSelectedSensorPosition(0);
        double leftVelUnitsPer100ms = _leftFront.getSelectedSensorVelocity(0);
        double rghtVelUnitsPer100ms = _rghtFront.getSelectedSensorVelocity(0);

        work += " L:" + leftVelUnitsPer100ms + " R:" + rghtVelUnitsPer100ms;

        /*
         * drive motor at least 25%, Talons will auto-detect if sensor is out of phase
         */
        _leftFront.getFaults(_faults_L);
        _rghtFront.getFaults(_faults_R);

        if (_faults_L.SensorOutOfPhase) {
            work += " L sensor is out of phase";
        }
        if (_faults_R.SensorOutOfPhase) {
            work += " R sensor is out of phase";
        }

        /* print to console if btn1 is held down */
        if (btn1) {
            System.out.println(work);
        }
    }

    @Override
    public void robotInit() {
        /* factory default values */
        _rghtFront.configFactoryDefault();
        _rghtFollower.configFactoryDefault();
        _leftFront.configFactoryDefault();
        _leftFollower.configFactoryDefault();

        /* set up followers */
        _rghtFollower.follow(_rghtFront);
        _leftFollower.follow(_leftFront);

        /* [3] flip values so robot moves forward when stick-forward/LEDs-green */
        _rghtFront.setInverted(TalonFXInvertType.Clockwise); // !< Update this
        _leftFront.setInverted(TalonFXInvertType.CounterClockwise); // !< Update this

        /*
         * set the invert of the followers to match their respective master controllers
         */
        _rghtFollower.setInverted(InvertType.FollowMaster);
        _leftFollower.setInverted(InvertType.FollowMaster);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // _rghtFront.setSensorPhase(true);
        // _leftFront.setSensorPhase(true);

        /*
         * WPI drivetrain classes defaultly assume left and right are opposite. call
         * this so we can apply + to both sides when moving forward. DO NOT CHANGE
         */
        _diffDrive.setRightSideInverted(false);
        
		SmartDashboard.putData("Field", _driveSim.getField());
    }
}
