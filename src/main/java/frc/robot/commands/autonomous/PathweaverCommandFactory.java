// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.nio.file.Path;
import java.nio.file.InvalidPathException;
import java.util.HashMap;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import frc.robot.Constants;
import frc.robot.utility.PIDGains;
import frc.robot.utility.SendableGains;
import frc.robot.utility.trajectoryLoader.LoadObserver;
import frc.robot.utility.trajectoryLoader.TrajectoryLoader;
import frc.robot.subsystems.driveBase.WestCoastDriveTrain;
import frc.robot.subsystems.driveBase.WCDriveConstants;

/**
 * A Command object that carries out a trajectory created with Pathweaver
 */
public class PathweaverCommandFactory implements LoadObserver {
  /** Reasonable baseline value in units of meters and seconds for a RAMSETE follower 'b' gain.
   * Larger values of b make convergence more aggressive like a proportional term.  For info
   * on tuning B gain, see:
   *    https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/ramsete.html#constructing-the-ramsete-controller-object
   */
  public static final double kRamseteB = 2;
  /** Reasonable baseline value in units of meters and seconds for a RAMSETE 'zeta' gain.
   * Larger values of zeta provide more damping in the response.  For info on tuning zeta gain,
   * see:
   *    https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/ramsete.html#constructing-the-ramsete-controller-object
   */
  public static final double kRamseteZeta = 0.7;

  /** A map of auto routine names to Trajectory objects */
  public final HashMap<String, Trajectory> m_trajectoryMap = new HashMap<String, Trajectory>();

  /** 
   * PID gains to use in conjunction with the Ramsete command used by Pathweaver commands
   */
  private final PIDGains m_gains = new PIDGains();

  /** Object used to make m_gains editable via the dashboard */
  private final SendableGains m_sendableGains = new SendableGains(m_gains);

  /** 
   * Initializes the factory with trajectories loaded from JSON files.  This constructor must be
   * called from the Robot.robotInit() routine, prior to the Scheduler starting to execute because
   * it performs disk I/O that can last multiple scheduler cycles.
   * 
   * @param jsonDir  Relative path of a directory containing Pathweaver JSON files
   */
  public PathweaverCommandFactory(String jsonDir) {
    try {
      Path jsonDirPath = Filesystem.getDeployDirectory().toPath().resolve(jsonDir);
      TrajectoryLoader.loadJSONFiles(jsonDirPath, this);
    } catch (InvalidPathException e) {
      String msg = String.format("Failed to open paths directory %s\n%s", jsonDir.toString(), e.getCause());
      DriverStation.reportError(msg, false);
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Gets an array containing the names of available Trajectories loaded from Pathweaver JSON files
   * @return An array containing the names of Trajectories loaded from Pathweaver JSON files
   */
  public String[] getTrajectoryNames() {
    return m_trajectoryMap.keySet().toArray(new String[m_trajectoryMap.size()]);
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Get the Trajectory that was loaded from a Pathweaver JSON file
   * @param name Name string returned from getTrajectoryNames()
   * @return Trajectory object corresponding to the given name
   */
  public Trajectory getTrajectory(String name) {
    return m_trajectoryMap.get(name);
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Returns a SendableGains object for adjusting PID gains associated with Pathweaver trajectories
   */
  public SendableGains getSendableGains() {
    return m_sendableGains;
  }

  /** 
   * Returns a Command object that will execute a given Trajectory created using Pathweaver
   * @param trajectory  Trajectory to create a command for
   * @param driveTrainSubsystem Drive train subsystem used to carry out the command
   */
  public Command createPathweaverCommand(Trajectory trajectory,
                                         WestCoastDriveTrain driveTrainSubsystem) {
    RamseteCommand ramseteCommand =
        new RamseteCommand(
            trajectory,
            driveTrainSubsystem::getPose,
            new RamseteController(kRamseteB, kRamseteZeta),
            new SimpleMotorFeedforward(
              WCDriveConstants.Kinematics.ksVolts,
              WCDriveConstants.Kinematics.kvVoltSecondsPerMeter,
              WCDriveConstants.Kinematics.kaVoltSecondsSquaredPerMeter),
            WCDriveConstants.Kinematics.kDriveKinematics,
            driveTrainSubsystem::getWheelSpeeds,
            new PIDController(m_gains.kP(), m_gains.kI(), m_gains.kD()),
            new PIDController(m_gains.kP(), m_gains.kI(), m_gains.kD()),
            // RamseteCommand passes volts to the callback
            driveTrainSubsystem::tankDriveVolts,
            driveTrainSubsystem);

    // Run the path following command after resetting the pose.
    // Stop the bot when the command has completed.
    return ramseteCommand
      .beforeStarting(() -> driveTrainSubsystem.resetPose(trajectory.getInitialPose()))
      .andThen(() -> driveTrainSubsystem.tankDriveVolts(0, 0));
  }


  /**
   * Method called when a Trajectory has been loaded successfully
   * 
   * @param fileName   Name of the file the trajectory was loaded from
   * @param trajectory Trajectory object that was loaded
   */
  public void onTrajectoryCreated(String fileName, Trajectory trajectory) {
    String trajectoryName = fileName.split("\\.")[0];
    m_trajectoryMap.put(trajectoryName, trajectory);
  }

  /**
   * Method called on failure to load the contents of a file into a
   * Trajectory object
   * 
   * @param filePath Path of the file that could not be loaded
   */
  public void onLoadFailed(Path filePath, String cause) {
    DriverStation.reportError("Failed to load trajectory " + filePath + ": " + cause, false);
  }
}
