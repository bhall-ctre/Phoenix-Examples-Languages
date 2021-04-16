// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMUSimCollection;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.VecBuilder;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  XboxController m_gamepad = new XboxController(0);
  XboxControllerSim m_gamepadSim = new XboxControllerSim(m_gamepad);

  WPI_TalonFX m_leftDrive = new WPI_TalonFX(0);
  WPI_TalonFX m_leftFollower = new WPI_TalonFX(1);
  WPI_TalonFX m_rightDrive = new WPI_TalonFX(2);
  WPI_TalonFX m_rightFollower = new WPI_TalonFX(3);

  // Object for simulated inputs into Talon.
  TalonFXSimCollection m_leftDriveSim = m_leftDrive.getSimCollection();
  TalonFXSimCollection m_rightDriveSim = m_rightDrive.getSimCollection();

  WPI_PigeonIMU m_pigeon = new WPI_PigeonIMU(1);
  // Object for simulated inputs into Pigeon.
  PigeonIMUSimCollection m_pigeonSim = m_pigeon.getSimCollection();

  //These numbers are an example AndyMark Drivetrain with some additional weight.  This is a fairly light robot.
  //Note you can utilize results from robot characterization instead of theoretical numbers.
  //https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html#introduction-to-robot-characterization
  final int kCountsPerRev = 2048;  //Encoder counts per revolution of the motor shaft.
  final double kSensorGearRatio = 1; //Gear ratio is the ratio between the *encoder* and the wheels.  On the AndyMark drivetrain, encoders mount 1:1 with the gearbox shaft.
  final double kGearRatio = 10.71; //Switch kSensorGearRatio to this gear ratio if encoder is on the motor instead of on the gearbox.
  final double kWheelRadiusInches = 3;
  final int k100msPerSecond = 10;

  //Simulation model of the drivetrain
  DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
    DCMotor.getFalcon500(2),  //2 Falcon 500s on each side of the drivetrain.
    kGearRatio,               //Standard AndyMark Gearing reduction.
    2.1,                      //MOI of 2.1 kg m^2 (from CAD model).
    26.5,                     //Mass of the robot is 26.5 kg.
    Units.inchesToMeters(kWheelRadiusInches),  //Robot uses 3" radius (6" diameter) wheels.
    0.546,                    //Distance between wheels is _ meters.
    
    // The standard deviations for measurement noise:
    // x and y:          0.001 m
    // heading:          0.001 rad
    // l and r velocity: 0.1   m/s
    // l and r position: 0.005 m
    null //VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005) //Uncomment this line to add measurement noise.
  );

  Field2d m_field = new Field2d();
  // Creating my odometry object. Here,
  // our starting pose is 5 meters along the long end of the field and in the
  // center of the field along the short end, facing forward.
  DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_pigeon.getRotation2d());

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_rightDrive.configFactoryDefault();
    m_rightFollower.configFactoryDefault();
    m_rightFollower.follow(m_rightDrive);
    m_rightFollower.setInverted(InvertType.FollowMaster);

    m_leftDrive.configFactoryDefault();
    m_leftFollower.configFactoryDefault();
    m_leftFollower.follow(m_leftDrive);
    m_leftFollower.setInverted(InvertType.FollowMaster);

    m_leftDrive.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    m_rightDrive.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    SmartDashboard.putData("Field", m_field);

    // On the robot, the left side is positive forward, so don't change it.
    m_leftDrive.setInverted(TalonFXInvertType.CounterClockwise);

    // On the robot, the right side output needs to be inverted so that positive is forward.
    m_rightDrive.setInverted(TalonFXInvertType.Clockwise);

    /*
     * Talon FX does not need sensor phase set for its integrated sensor
     * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
     * and the user calls getSelectedSensor* to get the sensor's position/velocity.
     * 
     * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
     */
    //m_leftDrive.setSensorPhase(false);
    //m_rightDrive.setSensorPhase(false);
  }

  @Override
  public void robotPeriodic() {
    // This will get the simulated sensor readings that we set
    // in the previous article while in simulation, but will use
    // real values on the robot itself.
    m_odometry.update(m_pigeon.getRotation2d(),
                      nativeUnitsToDistanceMeters(m_leftDrive.getSelectedSensorPosition()),
                      nativeUnitsToDistanceMeters(m_rightDrive.getSelectedSensorPosition()));
    m_field.setRobotPose(m_odometry.getPoseMeters());
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    double throttle = -m_gamepad.getY(Hand.kLeft);
    double turn = m_gamepad.getX(Hand.kRight);

    //Basic Arcade Drive.  This can optionally be replaced with WPILib's DifferentialDrive class.
    m_leftDrive.set(ControlMode.PercentOutput, throttle, DemandType.ArbitraryFeedForward, turn);
    m_rightDrive.set(ControlMode.PercentOutput, throttle, DemandType.ArbitraryFeedForward, -turn);
  }

  @Override 
  public void simulationPeriodic() {
    // Set the inputs to the system. Note that we need to use
    // the output voltage, NOT the percent output.
    m_driveSim.setInputs(m_leftDriveSim.getMotorOutputLeadVoltage(),
                         -m_rightDriveSim.getMotorOutputLeadVoltage()); //Right side is inverted, so forward is negative voltage

    // Advance the model by 20 ms. Note that if you are running this
    // subsystem in a separate thread or have changed the nominal timestep
    // of TimedRobot, this value needs to match it.
    m_driveSim.update(0.02);

    // Update all of our sensors.
    m_leftDriveSim.setIntegratedSensorRawPosition(
                    distanceToNativeUnits(
                    m_driveSim.getLeftPositionMeters()));
    m_leftDriveSim.setIntegratedSensorVelocity(
                    velocityToNativeUnits(
                    m_driveSim.getLeftVelocityMetersPerSecond()));
    m_rightDriveSim.setIntegratedSensorRawPosition(
                    distanceToNativeUnits(
                    -m_driveSim.getRightPositionMeters()));
    m_rightDriveSim.setIntegratedSensorVelocity(
                    velocityToNativeUnits(
                    -m_driveSim.getRightVelocityMetersPerSecond()));
    m_pigeonSim.setRawHeading(m_driveSim.getHeading().getDegrees());

    //Update other inputs to Talons
    m_leftDriveSim.setBusVoltage(RobotController.getBatteryVoltage());
    m_rightDriveSim.setBusVoltage(RobotController.getBatteryVoltage());
  }


  private int distanceToNativeUnits(double positionMeters){
    double wheelRotations = positionMeters/(2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
    double motorRotations = wheelRotations * kSensorGearRatio;
    int sensorCounts = (int)(motorRotations * kCountsPerRev);
    return sensorCounts;
  }

  private int velocityToNativeUnits(double velocityMetersPerSecond){
    double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
    double motorRotationsPerSecond = wheelRotationsPerSecond * kSensorGearRatio;
    double motorRotationsPer100ms = motorRotationsPerSecond / k100msPerSecond;
    int sensorCountsPer100ms = (int)(motorRotationsPer100ms * kCountsPerRev);
    return sensorCountsPer100ms;
  }

  private double nativeUnitsToDistanceMeters(double sensorCounts){
    double motorRotations = (double)sensorCounts / kCountsPerRev;
    double wheelRotations = motorRotations / kSensorGearRatio;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
    return positionMeters;
  }
}
