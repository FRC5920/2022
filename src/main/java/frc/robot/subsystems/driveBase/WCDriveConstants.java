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


import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

import edu.wpi.first.math.util.Units;

import frc.robot.utility.PIDGains;

/**
 * Constants used in conjunction with the WestCoastDrivetrain class
 */
public class WCDriveConstants {

  /**
   * Constants for motors used in the drive train
   */
  public static final class Motors {
    /** CAN ID of the left master motor */
    public static final int kLeftMasterCANid = 2; // Falcon 500
    /** CAN ID of the right master motor */
    public static final int kRightMasterCANid = 1; // Falcon 500

    /** The number of motors used in the drive base subsystem */
    public static final int kNumMotors = 2;

    /**
     * Ramp time applied by motor controllers, given as the number of seconds to transition from 0
     * Amps to full current. Greater values can help to limit motor current draw, but may result in
     * less responsive control.
     */
    public static final double kMotorRampTimeSec = 0.1;
  }

  /**
   * Physical constants describing the drive train (Imperial units)
   * 
   **************************************************************************
   * NOTE: Constants provided in this class are for poor souls still measuring in IMPERIAL UNITS
   * (fingers in mouth with gagging noises...). Physical dimensions of drive base components entered
   * here are automatically converted to equivalent SI unit values in constants below.
   ************************************************************************** 
   */
  private static final class PhysicalImp {
    /** Width of the west coast drive tracks in inches */
    public static final double kTrackWidthInches = 22.0; // Measured 5/31/2022

    /** Diameter of wheels in the west coast drive train (inches) */
    public static final double kWheelDiameterInches = 6.0; // Measured 5/31/2022

    /** Gear ratio applied between drive motors and wheels */
    public static final double kGearRatio = 8.45; // (8.45:1 gear ratio - Defined 2/27/2022)
  }

  /**
   * Physical constants describing the drive train (Imperial units)
   * 
   * These constants are automatically calculated from values given in the PhysicalImp class above.
   * They are not expected to be edited directly.
   */
  public static final class PhysicalSI {
    /** Width of the west coast drive tracks in meters */
    public static final double kTrackWidthMeters = Units
        .inchesToMeters(PhysicalImp.kTrackWidthInches);
    /** Radius of wheels in meters */
    public static final double kWheelDiameterMeters = Units
        .inchesToMeters(PhysicalImp.kWheelDiameterInches);
    /** Gear ratio applied between drive motors and wheels */
    public static final double kGearRatio = PhysicalImp.kGearRatio;

    /** Number of wheel encoder ticks per revolution in a Falcon motor */
    public static final int kFalconTicksPerRevolution = 2048; // Falcon encoder tick count
    /** Distance (meters) represented by each drive motor encoder tick */
    public static final double kMetersPerEncoderTick = (kWheelDiameterMeters * Math.PI)
        / ((double) kFalconTicksPerRevolution * kGearRatio);

  }

  /**
   * Constants used in conjunction with the drive base kinematic model.
   * 
   * These constants must be empirically determined using the sysid utility and by following
   * instructions provided at:
   * https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/characterizing-drive.html
   */
  public static final class Kinematics {
    /** Feed-forward static gain (ks) */
    public static final double ksVolts = 0.61949;
    /** Feed-forward velocity gain (kv) */
    public static final double kvVoltSecondsPerMeter = 2.0162;
    /** Feed-forward acceleration gain (kv) */
    public static final double kaVoltSecondsSquaredPerMeter = .46034;

    /** Maximum speed the robot should move at (meters per second) */
    public static final double kMaxSpeedMetersPerSec = 1.0;
    /** Maximum angular speed the robot should have (rotations per second) */
    public static final double kMaxAngularSpeedRotPerSec = 0.5 * Math.PI; // 0.5 rotation per second

    /** Kinematic model for the drive train */
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(PhysicalSI.kTrackWidthMeters);
  }

  /** 
   * Constants for use in trajectory-based autonomous routines.  See WPILib documentation for these
   * constants presented here:
   *    https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/entering-constants.html#max-trajectory-velocity-acceleration
   */
  public static final class Autonomous {
    /** Maximum speed (meters/second).  Set this slightly below nominal free speed of the bot */
    public static final double kMaxSpeedMetersPerSecond = 1.0;
    /** 
     * Maximum acceleration (meters/second^2).  The value of this constant becomes less critical
     * when a DifferentialDriveVoltageConstraint is applied in the auto routine.
     */
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;

    /** Voltage constraint applied to autonomous routines to limit acceleration */
    public static final DifferentialDriveVoltageConstraint kAutoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                      Kinematics.ksVolts,
                      Kinematics.kvVoltSecondsPerMeter,
                      Kinematics.kaVoltSecondsSquaredPerMeter),
                    Kinematics.kDriveKinematics,
                    10);
    
        // Create config for trajectory
    public static final TrajectoryConfig kTrajectoryConfig =
            new TrajectoryConfig(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(Kinematics.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(kAutoVoltageConstraint);
  }

  /**
   * Initial PID Controller Gains for left side of drive train
   */
  public static final PIDGains kDefaultLeftGains = new PIDGains(
      false, // true to enable PID control
      1.0, // Proportional gain (kP)
      0.0, // Integral gain (kI)
      0.0, // Derivative gain (kD)
      0.0 // Feed-forward gain (kF)
  );

  /**
   * Initial PID Controller Gains for right side of drive train
   */
  public static final PIDGains kDefaultRightGains = new PIDGains(
      false, // true to enable PID control
      1.0, // Proportional gain (kP)
      0.0, // Integral gain (kI)
      0.0, // Derivative gain (kD)
      0.0 // Feed-forward gain (kF)s
  );
}
