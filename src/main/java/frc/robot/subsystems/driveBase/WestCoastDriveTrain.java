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

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import frc.robot.utility.PIDGains;
import frc.robot.utility.SendableGains;

///////////////////////////////////////////////////////////////////////////////
/**
 * A subsystem for managing drive base motors and odometry.
 * 
 * Based on the WPILib drive base subsystem given with the Ramsete Command example project:
 * https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/ramsetecommand
 */
public class WestCoastDriveTrain extends SubsystemBase {
  private static final int kLeft = 0;
  private static final int kRight = 0;

  //////////////////////////////////
  /// Drive train motors
  //////////////////////////////////

  /** The motors on the left side of the drive train. Master is at index 0. */
  private final WPI_TalonFX[] m_leftFalcons = {
      new WPI_TalonFX(WCDriveConstants.Motors.kLeftMasterCANid) };
  /** Motor controller group used to address motors on left side of drive train */
  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftFalcons);

  /** The motors on the right side of the drive train. Master is at index 0. */
  private final WPI_TalonFX[] m_rightFalcons = {
      new WPI_TalonFX(WCDriveConstants.Motors.kRightMasterCANid) };
  /** Motor controller group used to address motors on left side of drive train */
  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightFalcons);

  //////////////////////////////////
  /// Motor Control
  //////////////////////////////////

  /** Index of active Falcon PID configuration */
  private static final int m_PIDIndex = 0;

  /** PID controller gains used for closed-loop velocity control in drive train motors */
  private PIDGains m_motorGains[] = { new PIDGains(WCDriveConstants.kDefaultLeftGains),
      new PIDGains(WCDriveConstants.kDefaultRightGains) };

  /** Object used to make left PID controller gains editable via Shuffleboard widgets */
  private SendableGains m_sendableGains[] = { new SendableGains(m_motorGains[kLeft]),
      new SendableGains(m_motorGains[kRight]) };

  /**
   * PID velocity error values from motor controllers. These are presented in sensor units per
   * controller period. By default on a CTRE Falcon, units are control_ticks per 0.1 sec.
   */
  private double m_velocitySensorError[] = { 0, 0 };

  /** Differential drive controller used to drive motors */
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  //////////////////////////////////////////
  // Odometry
  //////////////////////////////////////////

  /** Odometry class for tracking robot pose */
  private final DifferentialDriveOdometry m_odometry;
  /** Gyro sensor referenced for odometry */
  private final Gyro m_gyro = new ADXRS450_Gyro();

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
   * This routine is called periodically from the scheduler at a nominal period of 20 ms
   */
  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    updateOdometry();

    // Update the present velocity error values from master motor controllers
    m_velocitySensorError[kLeft] = m_leftFalcons[0].getClosedLoopError(m_PIDIndex);
    m_velocitySensorError[kRight] = m_rightFalcons[0].getClosedLoopError(m_PIDIndex);
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the current wheel speeds of the robot
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    double velocity[] = getEncoderVelocitySI();
    return new DifferentialDriveWheelSpeeds(velocity[0], velocity[1]);
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Updates the field-relative position using odometry measurements
   */
  public void updateOdometry() {
    //var gyroAngle = Rotation2d.fromDegrees(-1.0 * m_gyro.getAngle());
    Rotation2d gyroAngle = m_gyro.getRotation2d();
    double distance[] = getSensorDistanceMeters();
    m_odometry.update(gyroAngle, distance[0], distance[1]);

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
   * @param xSpeed       The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param zRotation    The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *                     positive.
   * @param squareInputs If set, decreases the input sensitivity at low speeds.
   */
  public void arcadeDrive(double xSpeed, double zRotationRate, boolean useLowSensitivity) {
    m_diffDrive.arcadeDrive(xSpeed, zRotationRate, useLowSensitivity);
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Controls the left and right sides of the drive directly using proportional values (-1.0 to 1.0)
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
   * Called by a RamseteCommand in autonomous routines to controls the left and right sides of the
   * drive train directly using voltage values.
   *
   * @param leftVolts  Voltage applied to motors on the left side of the drive train
   * @param rightVolts  Voltage applied to motors on the right side of the drive train
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {    
    SmartDashboard.putNumber("Left Volts", leftVolts);
  SmartDashboard.putNumber("Right Volts", rightVolts);
  SmartDashboard.putNumber("Left velocityspd", m_leftFalcons[0].getSelectedSensorPosition());
  SmartDashboard.putNumber("Right velocityspd", m_rightFalcons[0].getSelectedSensorPosition());
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_diffDrive.feed();
  }
  
  /////////////////////////////////////////////////////////////////////////////
  /**
   * Resets drive base encoders to zero on the current position
   */
  public void resetEncoders() {
    m_rightFalcons[0].setSelectedSensorPosition(0, 0, 0);
    m_leftFalcons[0].setSelectedSensorPosition(0, 0, 0);
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Returns the average distance (in meters) indicated by left and right side encoders */
  public double getAverageSensorDistanceMeters() {
    double sensorDistance[] = getSensorDistanceMeters();
    return (sensorDistance[kLeft] + sensorDistance[kRight]) / 2.0;
  }

  /**
   * Returns the sensor distance (meters) from the left and right drive train motors
   * 
   * @return An array containing sensor distances, with elements 0=left 1=right
   */
  public double[] getSensorDistanceMeters() {
    double ticks[] = getEncoderDistance();
    double distance[] = { falconEncoderDistanceToSI(ticks[0]), falconEncoderDistanceToSI(ticks[1]) };
    return distance;
  }

  /**
   * Returns the sensor distance (encoder ticks) from the left and right drive train motors
   * 
   * @return An array containing sensor distances, with elements 0=left 1=right
   */
  private double[] getEncoderDistance() {
    double ticks[] = { m_leftFalcons[0].getSelectedSensorPosition(),
                       -1.0 * m_rightFalcons[0].getSelectedSensorPosition() };
    return ticks;
  }

  /**
   * Returns the sensor velocity (meters per second) from the left and right drive train motors
   * 
   * @return An array containing sensor velocities, with elements 0=left 1=right
   */
  public double[] getEncoderVelocitySI() {
    double velocityTicks[] = getEncoderVelocity();
    return new double[]{ falconEncoderVelocityToSI(velocityTicks[0]),
                         falconEncoderVelocityToSI(velocityTicks[1]) };
  }

  /**
   * Returns the sensor velocity (encoder ticks per 100 ms) from the left and right drive train motors
   * 
   * @return An array containing sensor velocities, with elements 0=left 1=right
   */
  private double[] getEncoderVelocity() {
    double ticks[] = { m_leftFalcons[0].getSelectedSensorVelocity(),
                       -1.0 * m_rightFalcons[0].getSelectedSensorVelocity() };
    return ticks;
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
  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_diffDrive.setMaxOutput(maxOutput);
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Returns the turn rate of the robot in degrees per second */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the closed loop velocity control error of the left and right sides of the drive train
   * 
   * @return An array of velocity control error in sensor ticks per second (0=left, 1=right)
   */
  public double[] getVelocitySensorError() {
    double errorPerSec[] = { m_velocitySensorError[kLeft] * 10,
        m_velocitySensorError[kRight] * 10, };

    return errorPerSec;
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Returns an array of SendableGains for the drive train with 0=left, 1=right
   */
  public SendableGains[] getSendableGains() {
    return m_sendableGains;
  }

  /**
   * Drive train motor stop behaviors
   */
  public enum MotorStopBehavior {
    /** Motors brake to a stop when requested speed/Volts/percent are zero */
    Brake,
    /** Motors coast on their inertia when requested speed/Volts/percent are zero  */
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

    m_leftFalcons[0].setNeutralMode(mode);
    m_rightFalcons[0].setNeutralMode(mode);
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Enable/disable motor safety for drive train motors
   * 
   * @param enabled true if motor safety is enforced for drive train motors
   */
  public void setMotorSafetyEnabled(boolean shouldEnable) {
    m_leftFalcons[0].setSafetyEnabled(shouldEnable);
    m_rightFalcons[0].setSafetyEnabled(shouldEnable);
  }

  /////////////////////////////////////////////////////////////////////////////
  public void limitMotors(boolean shouldLimit) {
    // Configure current limiting
    DriveTrainPowerBudget budget = new DriveTrainPowerBudget(200,
        WCDriveConstants.Motors.kNumMotors, 0.0);

    SupplyCurrentLimitConfiguration limitConfig = new SupplyCurrentLimitConfiguration(shouldLimit, // Enable/disable
                                                                                                   // current
                                                                                                   // limiting
        budget.motorLimitAmps(), // Current limit to apply (Amps)
        budget.motorLimitThresholdAmps(), // Threshold for current limiting
        budget.kMotorCurrentLimitHoldoffSec); // Time to wait before applying current limiting

    for (WPI_TalonFX motor : m_leftFalcons) {
      motor.configSupplyCurrentLimit(limitConfig);
      motor.configOpenloopRamp(WCDriveConstants.Motors.kMotorRampTimeSec);
    }
    
    for (WPI_TalonFX motor : m_rightFalcons) {
      motor.configSupplyCurrentLimit(limitConfig);
      motor.configOpenloopRamp(WCDriveConstants.Motors.kMotorRampTimeSec);
    } 
  }

  /////////////////////////////////////////////////////////////////////////////
  /** 
   * Applies PID controller gains to the left side of the drive train
   */
  public void setLeftMotorGains() {
    applyMotorPIDGains(m_leftFalcons[0], m_motorGains[kLeft], m_PIDIndex);
  }

  /////////////////////////////////////////////////////////////////////////////
  /** 
   * Applies PID controller gains to the right side of the drive train
   */
  public void setRightMotorGains() {
    applyMotorPIDGains(m_rightFalcons[0], m_motorGains[kRight], m_PIDIndex);
  }


  /////////////////////////////////////////////////////////////////////////////
  /**
   * Converts falcon sensor distance (ticks) to SI units (meters)
   * 
   * @param sensorTicks Falcon internal sensor encoder distance (encoder ticks)
   */
  private static double falconEncoderDistanceToSI(double sensorTicks) {
    return sensorTicks * WCDriveConstants.PhysicalSI.kMetersPerEncoderTick;
  }

    /////////////////////////////////////////////////////////////////////////////
  /**
   * Converts a closed-loop velocity sensor value to SI units (meters/sec)
   * 
   * @param sensorVelocity Falcon internal sensor encoder velocity (ticks per 100ms)
   */
  private static double falconEncoderVelocityToSI(double sensorVelocity) {
    double ticksPerSecond = sensorVelocity * 10; // Convert to ticks per sec
    return ticksPerSecond * WCDriveConstants.PhysicalSI.kMetersPerEncoderTick;
  }

    /////////////////////////////////////////////////////////////////////////////
  /** Configures drive subsystem motors */
  private void ConfigureMotors() {
    for (WPI_TalonFX motor : m_leftFalcons) {
      motor.configFactoryDefault();
    }

    for (WPI_TalonFX motor : m_rightFalcons) {
      motor.configFactoryDefault();
    }

    // Apply motor current limiting
    limitMotors(true);

    // TODO: apply closed-loop gains to Falcon motors

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotors.setInverted(true);
  }
  
  /////////////////////////////////////////////////////////////////////////////
  /**
   * Apply a Gains object to a Falcon motor
   * 
   * @param motor Falcon motor to configure
   * @param gains Gains to apply to the motor
   */
  private static void applyMotorPIDGains(WPI_TalonFX motor, PIDGains gains, int pidIndex) {
    // Timeout value (in milliseconds) used for commands used to configure the
    // motor. If nonzero, config functions will block while waiting for motor
    // configuration to succeed, and report an error if configuration times out.
    // If zero, no blocking or error checking is performed.
    final int kConfigTimeoutMs = 30;

    motor.config_kF(pidIndex, gains.kF(), kConfigTimeoutMs);
    motor.config_kP(pidIndex, gains.kP(), kConfigTimeoutMs);
    motor.config_kI(pidIndex, gains.kI(), kConfigTimeoutMs);
    motor.config_kD(pidIndex, gains.kD(), kConfigTimeoutMs);
  }

}