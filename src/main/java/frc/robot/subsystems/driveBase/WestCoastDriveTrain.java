///////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2022 FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
///////////////////////////////////////////////////////////////////////////////////////////////////

/*-----------------------------------------------------------------------------\
|                                                                              |
|                       ================================                       |
|                       **    TEAM 5290 - Vikotics    **                       |
|                       ================================                       |
|                                                                              |
|                            °        #°                                       |
|                            *O       °@o                                      |
|                            O@ °o@@#° o@@                                     |
|                           #@@@@@@@@@@@@@@                                    |
|                           @@@@@@@@@@@@@@@                                    |
|                           @@@@@@@@@@@@@@°                                    |
|                             #@@@@@@@@@@@@@O....   .                          |
|                             o@@@@@@@@@@@@@@@@@@@@@o                          |
|                             O@@@@@@@@@@@@@@@@@@@#°                    *      |
|                             O@@@@@@@@@@@@@@@@@@@@@#O                O@@    O |
|                            .@@@@@@@@°@@@@@@@@@@@@@@@@#            °@@@    °@@|
|                            #@@O°°°°  @@@@@@@@@@@@@@@@@@°          @@@#*   @@@|
|                         .#@@@@@  o#oo@@@@@@@@@@@@@@@@@@@@@.       O@@@@@@@@@@|
|                        o@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@°     @@@@@@@@@°|
|                        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@   .@@@@@o°   |
|          °***          @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@  @@@@@o     |
|     o#@@@@@@@@@@@@.   *@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@o@@@@@@      |
|OOo°@@@@@@@@@@@@O°#@#   @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@       |
|@@@@@@@@@@@@@@@@    o°  .@@@@@@@@@@@@@@@@@@@@@@@@#*@@@@@@@@@@@@@@@@@@@@       |
|@@@@@@@@@@@@@@@*         O@@@@@@@@@@@@@@@@@@@@@@@   °@@@@@@@@@@@@@@@@@@o      |
|@@@@#@@@@@@@@@            @@@@@@@@@@@@@@@@@@@@@@       .*@@@@@@@@@@@@@@.      |
|@@@°      @@@@O           @@@@@@@@@@@@@@@@@@@@o           °@@@@@@@@@@@o       |
|          @@@@@          .@@@@@@@@@@@@@@@@@@@*               O@@@@@@@*        |
|           @@@@@        o@@@@@@@@@@@@@@@@@@@@.               #@@@@@O          |
|           *@@@@@@@*  o@@@@@@@@@@@@@@@@@@@@@@°              o@@@@@            |
|           @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.              @@@@@#            |
|          @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@O             #@@@@@             |
|          .@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@#           .@@@@@°             |
|           @@@@@@@@@@O*    @@@@@@@@@@@@@@@@@@@@@°         °O@@@°              |
|            °O@@@@@@       @@@@@@@@@@@@@@@@@@@@@@@                            |
|              o@@@@@°      @@@@@@@@@@@@@@@@@@@@@@@@                           |
|               @@@@@@.     @@@@@@@@@@@@@@@@@@@@@@@@@o                         |
|                @@@@@@*    @@@@@@@@@@@@@@@@@@@@@@@@@@                         |
|                o@@@@@@.  o@@@@@@@@@@@@@@@@@@@@@@@@@@@                        |
|                 #@@@@@@  *@@@@@@@@@@@@@@@@@@@@@@@@@@@@                       |
|                  °***    @@@@@@@@@@@@@@@@@@@@@@@@@@@@@O                      |
|                         .OOOOOOOOOOOOOOOOOOOOOOOOOOOOOO                      |
\-----------------------------------------------------------------------------*/

package frc.robot.subsystems.driveBase;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import frc.robot.Constants;
import frc.robot.utility.Gains;

///////////////////////////////////////////////////////////////////////////////
/**
 * A subsystem for managing drive base motors and odometry.
 * 
 * Based on the WPILib drive base subsystem given with the Ramsete Command example project:
 * https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/ramsetecommand
 */
public class WestCoastDriveTrain extends SubsystemBase {
  /** The number of motors used in the drive base subsystem */
  public static final int kNumDriveMotors = 2;

  /** 
   * Ramp time applied to motor current limiting, defined as the number of seconds to transition
   * from 0 Amps to full current
   */
  public static final double kMotorRampTimeSec = 0.05;

  /////////////////////////////////////
  // Kinematics/Odometry Constants
  /////////////////////////////////////

  /** Maximum speed the robot should move at (meters per second) */
  public static final double kMaxSpeedMetersPerSec = 3.0;
  /** Maximum angular speed the robot should have (rotations per second) */
  public static final double kMaxAngularSpeedRotPerSec = 1 * Math.PI; // Half a rotation per second

  // ------ Physical Constants for Odometry (inches) ------
  /** Width of the west coast drive tracks in inches */
  private static final double kTrackWidthInches = 27.16535; // Measured 2/27/2022
  /** Diameter of wheels in the west coast drive train (inches) */
  private static final double kWheelDiameterInches = 2.004; // Measured 2/27/2022
  /** Gear ratio applied between drive motors and wheels */
  public static final double kGearRatio = 8.45; // Defined 2/27/2022


  // ------ Physical Constants for Odometry (meters) ------
  // NOTE:
  // These constants are automatically calculated from values in inches as a convenience. They are
  // not expected to be edited directly

  /** Width of the west coast drive tracks in meters */
  private static final double kTrackWidthMeters = Units.inchesToMeters(kTrackWidthInches);
  /** Radius of wheels in meters */
  private static final double kWheelDiameterMeters = Units.inchesToMeters(kWheelDiameterInches);
  /** Number of wheel encoder ticks per revolution in a Falcon motor */
  private static final int kFalconTicksPerRevolution = 2048; // Falcon encoder tick count
  /** Distance (meters) represented by each drive motor encoder tick */
  private static final double kMetersPerEncoderTick = 
    (kWheelDiameterMeters * Math.PI) / (double) kFalconTicksPerRevolution * kGearRatio;


  //////////////////////////////////////////
  // Drive base controller gains
  //////////////////////////////////////////

  // Feed-forward gains used for drive base motors
  /** Feed-forward static gain (ks) */
  public static final double kFeedForward_ks = 0.0; 
  /** Feed-forward velocity gain (kv) */
  public static final double kFeedForward_kv = 0.0; 
  /** Feed-forward acceleration gain (kv) */
  public static final double kFeedForward_ka = 0.0; 

  /** 
   * Default PID controller gains used for the left side of the drive train
   */
  private static final Gains kLeftMotorGains = 
    new Gains( 0.0, // Kp
               0.0, // Ki
               0.0, // Kd
               0.0, // Kf (feed-forward)
               0,   // Izone
               0.0  // Peak output
             );

  /** 
   * Default PID controller gains used for the right side of the drive train
   */
  private static final Gains kRightMotorGains = 
    new Gains( 0.0, // Kp
               0.0, // Ki
               0.0, // Kd
               0.0, // Kf (feed-forward)
               0,   // Izone
               0.0  // Peak output
             );

  //////////////////////////////////////////
  // Kinematics/Odometry
  //////////////////////////////////////////

  /** Kinematic model for differential drive train */
  private final DifferentialDriveKinematics m_kinematics = 
    new DifferentialDriveKinematics(kTrackWidthMeters);
  /** Odometry class for tracking robot pose */
  private final DifferentialDriveOdometry m_odometry;
  /** Gyro sensor referenced for odometry */
  private final Gyro m_gyro = new ADXRS450_Gyro();


  //////////////////////////////////
  /// Drive train motors
  //////////////////////////////////

  // The motors on the left side of the drive base.
  private final WPI_TalonFX m_leftMaster = new WPI_TalonFX(Constants.MotorID.leftDriveMaster);
  private final MotorControllerGroup m_leftMotors = 
      new MotorControllerGroup(m_leftMaster);

  // The motors on the right side of the drive base.
  private final WPI_TalonFX m_rightMaster = new WPI_TalonFX(Constants.MotorID.rightDriveMaster);
  private final MotorControllerGroup m_rightMotors = 
      new MotorControllerGroup(m_rightMaster);

  /** PID controller gains for the left side of the drive train */
  @SuppressWarnings("unused")
  private Gains m_leftGains = kLeftMotorGains;

  /** PID controller gains for the right side of the drive train */
  @SuppressWarnings("unused")
  private Gains m_rightGains = kRightMotorGains;

  /** The robot's drive */
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  
  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_feedforward = 
      new SimpleMotorFeedforward(kFeedForward_ks, kFeedForward_kv, kFeedForward_ka);
  

  /////////////////////////////////////////////////////////////////////////////
  /** Creates an instance of the subsystem */
  public WestCoastDriveTrain() {
    ConfigureMotors();  // Configure drive train motors

    // Reset the gyro and encoders
    m_gyro.reset();
    resetEncoders();

    // Initialize odometry
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }

  

  /////////////////////////////////////////////////////////////////////////////
  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    updateOdometry();
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Sets the desired wheel speeds
   *
   * @param speeds The desired wheel speeds.
   */
  public void setWheelSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    m_leftMotors.setVoltage(speeds.leftMetersPerSecond + leftFeedforward);
    m_rightMotors.setVoltage(speeds.rightMetersPerSecond + rightFeedforward);
  }


  /////////////////////////////////////////////////////////////////////////////
  /** 
   * Returns the current velocity in meters per second of a given motor
   * @param motor  Motor whose velocity should be returned
   */
  private static double getMotorVelocity(WPI_TalonFX motor) {
    double ticksPer100ms = motor.getSelectedSensorVelocity();
    double ticksPerSecond = ticksPer100ms * 10;
    return ticksPerSecond * kMetersPerEncoderTick;
  }

  /////////////////////////////////////////////////////////////////////////////
  /** 
   * Returns the current wheel speeds of the robot
  */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds( getMotorVelocity(m_leftMaster),
                                             getMotorVelocity(m_rightMaster) );
  }

  
  /////////////////////////////////////////////////////////////////////////////
  /** 
   * Updates the field-relative position using odometry measurements
  */
  public void updateOdometry() {
    double leftDistance = m_leftMaster.getSelectedSensorPosition() * kMetersPerEncoderTick;
    double rightDistance = m_rightMaster.getSelectedSensorPosition() * kMetersPerEncoderTick;
    m_odometry.update(m_gyro.getRotation2d(), leftDistance, rightDistance);

    // NOTE: Additional measurements can be applied to the pose estimate
    // See the WPILib example provided at:
    // https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/differentialdriveposeestimator/Drivetrain.java
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Sets the pose to a specified value with the present gyro reading
   *
   * @param pose New pose to apply
   */
  public void resetPose(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Returns the current estimated pose of the robot */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd Forward movement being commanded
   * @param rot Rotation being commanded
   */
  public void arcadeDrive(double fwd, double rot) {
    m_diffDrive.arcadeDrive(fwd, rot);
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  Voltage to apply to the left side of the drive base
   * @param rightVolts Voltage to apply to the right side of the drive base
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_diffDrive.feed();
  }

  /////////////////////////////////////////////////////////////////////////////
  /** 
   * Resets drive base encoders to zero on the current position
   */
  public void resetEncoders() {
    m_rightMaster.setSelectedSensorPosition(0, 0, 0);
    m_leftMaster.setSelectedSensorPosition(0, 0, 0);
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Returns the average distance (in meters) indicated by left and right side encoders */
  public double getAverageEncoderDistance() {
    return (getLeftEncoderTicks() + getRightEncoderTicks()) * kMetersPerEncoderTick / 2.0;
  }

  /**
   * Returns the raw reading from the left drive encoder.
   * @return the left drive encoder
   */
  public double getLeftEncoderTicks() {
    return m_leftMaster.getSelectedSensorPosition();
  }

  /**
   * Returns the raw reading from the right drive encoder.
   * @return the right drive encoder
   */
  public double getRightEncoderTicks() {
    return m_rightMaster.getSelectedSensorPosition();
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_diffDrive.setMaxOutput(maxOutput);
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Returns the current heading of the robot in degrees (-180.0 to 180) */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Returns the turn rate of the robot in degrees per second */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot Angular velocity in rad/s.
  */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double rot) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setWheelSpeeds(wheelSpeeds);
  }

  /**
   * Stop behaviors for a motor
   */
  public enum MotorStopBehavior {
    /** Motor tends to brake to a stop */
    Brake,
    /** Motor coasts to a stop */
    Coast;
  };

  /////////////////////////////////////////////////////////////////////////////
  /** 
   * Sets drive motor neutral mode to determine behavior when stopping
  */
  public void setMotorStopBehavior(MotorStopBehavior behavior) {
    NeutralMode mode = NeutralMode.Brake;
    switch (behavior) {
      case Brake:
        break;

      case Coast:
        mode = NeutralMode.Coast;
        break;
    }

    m_rightMaster.setNeutralMode(mode);
    m_leftMaster.setNeutralMode(mode);
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Enable/disable motor safety for drive train motors
	 * @param enabled true if motor safety is enforced for drive train motors
  */
  public void setMotorSafetyEnabled(boolean shouldEnable) {
    m_leftMaster.setSafetyEnabled(shouldEnable);
    m_rightMaster.setSafetyEnabled(shouldEnable);
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Configures drive subsystem motors */
  private void ConfigureMotors() {
    m_leftMaster.configFactoryDefault();
    m_rightMaster.configFactoryDefault();

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotors.setInverted(true);
  }

  /////////////////////////////////////////////////////////////////////////////
  public void LimitMotors(boolean shouldLimit) {
    // Configure current limiting
    DriveTrainPowerBudget budget = new DriveTrainPowerBudget(200, kNumDriveMotors, 0.0);

    SupplyCurrentLimitConfiguration limitConfig = 
    new SupplyCurrentLimitConfiguration(shouldLimit, // Enable/disable current limiting
      budget.motorLimitAmps(),          // Current limit to apply (Amps)
      budget.motorLimitThresholdAmps(), // Threshold for current limiting
      budget.kMotorCurrentLimitHoldoffSec);   // Time to wait before applying current limiting

    m_leftMaster.configSupplyCurrentLimit(limitConfig);
    m_rightMaster.configSupplyCurrentLimit(limitConfig);

    // Configure current ramping (seconds required to ramp from neutral to full output)
    m_leftMaster.configOpenloopRamp(kMotorRampTimeSec);
    m_rightMaster.configOpenloopRamp(kMotorRampTimeSec);
  }
}