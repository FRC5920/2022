// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.driveBase.WCDriveConstants;


import frc.robot.RobotContainer;
import frc.robot.utility.PIDGains;
import frc.robot.utility.SendableGains;

/** Factory class used to create Commands for carrying out Autonomous routines */
public class AutoCommandFactory {
  ////////////////////////
  /// CONSTANTS
  ////////////////////////

  /**
   * Reasonable baseline value in units of meters and seconds for a RAMSETE follower 'b' gain.
   * Larger values of b make convergence more aggressive like a proportional term. For info on
   * tuning B gain, see:
   * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/ramsete.html#constructing-the-ramsete-controller-object
   */
  public static final double kRamseteB = 2;

  /**
   * Reasonable baseline value in units of meters and seconds for a RAMSETE 'zeta' gain. Larger
   * values of zeta provide more damping in the response. For info on tuning zeta gain, see:
   * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/ramsete.html#constructing-the-ramsete-controller-object
   */
  public static final double kRamseteZeta = 0.7;
  
  /**
   * PID gains used for tuning RamseteCommands
   */
  private static PIDGains s_gains = new PIDGains(
    true, // Enabled
    1.1182, // kP
    0.0,    // kI
    0.0,    // kD
    0.0     // kF
  );

  /** Object used to make m_gains editable via the dashboard */
  private static SendableGains s_sendableGains = new SendableGains(s_gains);

  /** 
   * Called to create a Command used to carry out an Autonomous routine
   * @param autoName  Name of the auto routine to create a Command for
   * @param botContainer  Container with robot subsystems
   */
  public static Command create(String autoName, RobotContainer botContainer) {


    // Create the named command
    switch (autoName) {
      // FUTURE:
      //   Specific command groups/sequences can be added here.  This allows for the construction
      //   of more complicated command implementations.

      // For now, we just fall through to the default case to create a Command from a loaded
      // PathWeaver Trajectory
      default:
      {
        Trajectory traj = botContainer.trajectoryStore.getTrajectory(autoName);
        return createPathWeaverCommand(traj, botContainer);
      }

    }
  }

  /**
   * Creates a Command from a PathWeaver trajectory
   * @param autoName  Name of the Trajectory to 
   * @return The Command and its Trajectory
   */
  private static Command createPathWeaverCommand(Trajectory trajectory, RobotContainer botContainer) {
    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, 
      botContainer.driveBaseSubsystem::getPose,
      new RamseteController(kRamseteB, kRamseteZeta),
      new SimpleMotorFeedforward(WCDriveConstants.Kinematics.ksVolts,
          WCDriveConstants.Kinematics.kvVoltSecondsPerMeter,
          WCDriveConstants.Kinematics.kaVoltSecondsSquaredPerMeter),
      WCDriveConstants.Kinematics.kDriveKinematics, 
      botContainer.driveBaseSubsystem::getWheelSpeeds,
      new PIDController(s_gains.kP(), s_gains.kI(), s_gains.kD()),
      new PIDController(s_gains.kP(), s_gains.kI(), s_gains.kD()),
      // RamseteCommand passes volts to the callback
      botContainer.driveBaseSubsystem::tankDriveVolts, 
      botContainer.driveBaseSubsystem);

// Run the path following command after resetting the pose.
// Stop the bot when the command has completed.
return ramseteCommand
    .beforeStarting(() -> botContainer.driveBaseSubsystem.resetPose(trajectory.getInitialPose()))
    .andThen(() -> botContainer.driveBaseSubsystem.tankDriveVolts(0, 0));
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Returns a SendableGains object for adjusting PID gains associated with Pathweaver trajectories
   */
  public static SendableGains getSendableGains() {
    return s_sendableGains;
  }

}
