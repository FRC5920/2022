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

import frc.robot.utility.Gains;
import frc.robot.utility.SendableGains;

///////////////////////////////////////////////////////////////////////////////
/**
 * A subsystem for managing drive base motors and odometry.
 * 
 * Based on the WPILib drive base subsystem given with the Ramsete Command example project:
 * https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/ramsetecommand
 */
public class WestCoastDriveTrain extends SubsystemBase {

  /**
   * Default PID controller gains used for the left side of the drive train
   */
  private SendableGains m_leftMotorGains = new SendableGains();

  /**
   * Default PID controller gains used for the right side of the drive train
   */
  private SendableGains m_rightMotorGains = new SendableGains();

  private static final int kPIDIndex = 0;

  //////////////////////////////////////////
  // Kinematics/Odometry
  //////////////////////////////////////////

  /** Kinematic model for differential drive train */
  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(
      WCDriveConstants.PhysicalSI.kTrackWidthMeters);
  /** Odometry class for tracking robot pose */
  private final DifferentialDriveOdometry m_odometry;
  /** Gyro sensor referenced for odometry */
  private final Gyro m_gyro = new ADXRS450_Gyro();

  // Possible Gear ratios:
  // 8.45:1
  //////////////////////////////////
  /// Drive train motors
  //////////////////////////////////

  // The motors on the left side of the drive base.
  private final WPI_TalonFX m_leftMaster = new WPI_TalonFX(
      WCDriveConstants.Motors.kLeftMasterCANid);
  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftMaster);

  // The motors on the right side of the drive base.
  private final WPI_TalonFX m_rightMaster = new WPI_TalonFX(
      WCDriveConstants.Motors.kRightMasterCANid);
  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightMaster);

  /** PID control errors received from motors */
  private final double m_motorError[] = new double[2];

  /** The robot's drive */
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(
    WCDriveConstants.Kinematics.ks, WCDriveConstants.Kinematics.kv, WCDriveConstants.Kinematics.ka);

  /////////////////////////////////////////////////////////////////////////////
  /** Creates an instance of the subsystem */
  public WestCoastDriveTrain() {
    ConfigureMotors(); // Configure drive train motors

    // Reset the gyro and encoders
    m_gyro.reset();
    resetEncoders();

    // Initialize odometry
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * This routine is called periodically from the scheduler at a nominal period
   * of 20 ms
   */
  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    updateOdometry();

    // Update the present motor error values
    m_motorError[0] = m_leftMaster.getClosedLoopError(kPIDIndex);
    m_motorError[1] = m_rightMaster.getClosedLoopError(kPIDIndex);
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
   * 
   * @param motor Motor whose velocity should be returned
   */
  private static double getMotorVelocity(WPI_TalonFX motor) {
    double ticksPer100ms = motor.getSelectedSensorVelocity();
    double ticksPerSecond = ticksPer100ms * 10;
    return ticksPerSecond * WCDriveConstants.PhysicalSI.kMetersPerEncoderTick;
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the current wheel speeds of the robot
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getMotorVelocity(m_leftMaster),
        getMotorVelocity(m_rightMaster));
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Updates the field-relative position using odometry measurements
   */
  public void updateOdometry() {
    double leftDistance = m_leftMaster.getSelectedSensorPosition()
        * WCDriveConstants.PhysicalSI.kMetersPerEncoderTick;
    double rightDistance = m_rightMaster.getSelectedSensorPosition()
        * WCDriveConstants.PhysicalSI.kMetersPerEncoderTick;
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
   * Controls the left and right sides of the drive directly with percentages.
   *
   * @param leftPercent  Percent of full scale drive to apply to the left side
   * @param rightPercent Percent of full scale drive to apply to the right side
   */
  public void tankDrivePercent(double leftPercent, double rightPercent) {
    m_leftMotors.set(leftPercent);
    m_rightMotors.set(rightPercent);
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
    return (getLeftEncoderTicks() + getRightEncoderTicks())
        * WCDriveConstants.PhysicalSI.kMetersPerEncoderTick / 2.0;
  }

  /**
   * Returns the raw reading from the left drive encoder.
   * 
   * @return the left drive encoder
   */
  public double getLeftEncoderTicks() {
    return m_leftMaster.getSelectedSensorPosition();
  }

  /**
   * Returns the raw reading from the right drive encoder.
   * 
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
   * Returns the closed loop error of the left drive train
   */
  public double getLeftMotorError() {
    return m_motorError[0];
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the closed loop error of the left drive train
   */
  public double getRightMotorError() {
    return m_motorError[1];
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Returns a reference to Gains used for the left side of the drive train
   */
  public SendableGains getLeftGains() {
    return m_leftMotorGains;
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Returns a reference to Gains used for the right side of the drive train
   */
  public SendableGains getRightGains() {
    return m_rightMotorGains;
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot    Angular velocity in rad/s.
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
   * 
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

    // TODO: apply closed-loop gains to Falcon motors

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotors.setInverted(true);
  }

  /////////////////////////////////////////////////////////////////////////////
  public void LimitMotors(boolean shouldLimit) {
    // Configure current limiting
    DriveTrainPowerBudget budget = new DriveTrainPowerBudget(200,
        WCDriveConstants.Motors.kNumMotors, 0.0);

    SupplyCurrentLimitConfiguration limitConfig = new SupplyCurrentLimitConfiguration(shouldLimit, // Enable/disable
                                                                                                   // current
                                                                                                   // limiting
        budget.motorLimitAmps(), // Current limit to apply (Amps)
        budget.motorLimitThresholdAmps(), // Threshold for current limiting
        budget.kMotorCurrentLimitHoldoffSec); // Time to wait before applying current limiting

    m_leftMaster.configSupplyCurrentLimit(limitConfig);
    m_rightMaster.configSupplyCurrentLimit(limitConfig);

    // Configure current ramping (seconds required to ramp from neutral to full output)
    m_leftMaster.configOpenloopRamp(WCDriveConstants.Motors.kMotorRampTimeSec);
    m_rightMaster.configOpenloopRamp(WCDriveConstants.Motors.kMotorRampTimeSec);
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Update left motor gains */
  public void updateLeftMotorGains() {
    applyMotorGains(m_leftMaster, m_leftMotorGains, kPIDIndex);
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Update left motor gains */
  public void updateRightMotorGains() {
    applyMotorGains(m_rightMaster, m_rightMotorGains, kPIDIndex);
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Apply a Gains object to a Falcon motor
   * 
   * @param motor Falcon motor to configure
   * @param gains Gains to apply to the motor
   */
  private static void applyMotorGains(WPI_TalonFX motor, Gains gains, int pidIndex) {
    // Timeout value (in milliseconds) used for commands used to configure the
    // motor. If nonzero, config functions will block while waiting for motor
    // configuration to succeed, and report an error if configuration times out.
    // If zero, no blocking or error checking is performed.
    final int kConfigTimeoutMs = 30;

    motor.config_kF(pidIndex, gains.kF, kConfigTimeoutMs);
    motor.config_kP(pidIndex, gains.kP, kConfigTimeoutMs);
    motor.config_kI(pidIndex, gains.kI, kConfigTimeoutMs);
    motor.config_kD(pidIndex, gains.kD, kConfigTimeoutMs);
    motor.config_IntegralZone(pidIndex, gains.iZone, kConfigTimeoutMs);
    motor.configClosedLoopPeakOutput(pidIndex, gains.peakOutput, kConfigTimeoutMs);
  }
}