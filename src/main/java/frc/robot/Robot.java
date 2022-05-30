///////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2022 FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
///////////////////////////////////////////////////////////////////////////////////////////////////

/*-------------------------------------------------------------------------------------------------\
|                                                                                                  |
|                                 ================================                                 |
|                                 **    TEAM 5290 - Vikotics    **                                 |
|                                 ================================                                 |
|                                                                                                  |
|                                  .             o°                                                |
|                                   °o          @@                                                 |
|                                    @@         o@@o                                               |
|                                   *@@ **@@@@** o@@*                                              |
|                                  *@@@@@@@@@@@@@@@@@*                                             |
|                                  @@@@@@@@@@@@@@@@@@@                                             |
|                                  @@@@@@@@@@@@@@@@@@*                                             |
|                                  @@@@@@@@@@@@@@@@@@                                              |
|                                   **@@@@@@@@@@@@@@@@**                                           |
|                                     @@@@@@@@@@@@@@@@@@@@@@@@***@                                 |
|                                     @@@@@@@@@@@@@@@@@@@@@@@@@@@o                                 |
|                                     @@@@@@@@@@@@@@@@@@@@@@@@oo                          **       |
|                                     @@@@@@@@@@@@@@@@@@@@@@@@@@@**                     *@@     *  |
|                                    *@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*                 *@@@O     @@ |
|                                    @@@@@@@@@@#°@@@@@@@@@@@@@@@@@@@@@               *@@@@     *@@@|
|                                   *@@@o°°°°°   @@@@@@@@@@@@@@@@@@@@@@*             @@@@o*    @@@@|
|                                 *@@@@@@    .  .@@@@@@@@@@@@@@@@@@@@@@@@@*          @@@@@@@@@@@@@@|
|                               *@@@@@@@@.o#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@**        @@@@@@@@@@@@@|
|                              *@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*      o@@@@@@@@@@* |
|                              @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@     @@@@@@@**    |
|                              @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*  @@@@@@@       |
|          ***@@@@@****        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ *@@@@@@*       |
|      *@@@@@@@@@@@@@@@@@*    *@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*@@@@@@@        |
|**** @@@@@@@@@@@@@@@@**@@@    @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@         |
|@@@@@@@@@@@@@@@@@@@@     o@    @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@         |
|@@@@@@@@@@@@@@@@@@@@      *    *@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@  *@@@@@@@@@@@@@@@@@@@@@@@@.        |
|@@@@@@@@@@@@@@@@@@@*            @@@@@@@@@@@@@@@@@@@@@@@@@@@@@*    *@@@@@@@@@@@@@@@@@@@@@@#        |
|@@@@@@@@@@@@@@@@@@              O@@@@@@@@@@@@@@@@@@@@@@@@@@@*        **@@@@@@@@@@@@@@@@@@o        |
|@@@@@  ******@@@@@               @@@@@@@@@@@@@@@@@@@@@@@@@@*            *@@@@@@@@@@@@@@@@         |
|****         @@@@@@              @@@@@@@@@@@@@@@@@@@@@@@@@                **@@@@@@@@@@@@          |
|             @@@@@@             *@@@@@@@@@@@@@@@@@@@@@@@@                    @@@@@@@@@@           |
|             *@@@@@@           @@@@@@@@@@@@@@@@@@@@@@@@@@                    @@@@@@@@*            |
|              @@@@@@****     *@@@@@@@@@@@@@@@@@@@@@@@@@@@                   @@@@@@@*              |
|               @@@@@@@@@@  *@@@@@@@@@@@@@@@@@@@@@@@@@@@@@                  *@@@@@@@               |
|              @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@                  @@@@@@@*               |
|             @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@                 *@@@@@@@                |
|             @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@                @@@@@@@                 |
|              @@@@@@@@@@@@@@@@@@**#@@@@@@@@@@@@@@@@@@@@@@@@              o@@@@@@*                 |
|              @@@@@@@@@@@@@**     @@@@@@@@@@@@@@@@@@@@@@@@@@*            oO@@@@*                  |
|               *@@@@@@@@@         @@@@@@@@@@@@@@@@@@@@@@@@@@@@*                                   |
|                  @@@@@@@         @@@@@@@@@@@@@@@@@@@@@@@@@@@@@*                                  |
|                  *@@@@@@@        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*                                 |
|                   @@@@@@@@       @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@                                |
|                    @@@@@@@*      @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*                               |
|                    @@@@@@@@@    #@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@                               |
|                     @@@@@@@@*   O@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@                              |
|                      @@@@@@@@   °@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@                             |
|                      *@@@@@*    @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@o                            |
|                                @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@                            |
|                                oooooooooooooooooooooooooooooooooooooo                            |
\-------------------------------------------------------------------------------------------------*/

package frc.robot;

import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.Filesystem;

import frc.robot.utility.trajectoryLoader.HashMapLoader;
import frc.robot.utility.trajectoryLoader.TrajectoryLoader;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  // *** Pathweaver */
  // String trajectoryJSON = "paths/YourPath.wpilib.json";
  // Trajectory trajectory = new Trajectory();
  // **--End Pathweaver */
  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    
    // ------------------------------
    // Load PathWeaver Data
    // ------------------------------
    // Create Trajectory objects from JSON files in the 'paths' directory
    // Reference: Importing a PathWeaver JSON
    // https://docs.wpilib.org/en/stable/docs/software/pathplanning/pathweaver/integrating-robot-program.html
    Path pathsDir = Filesystem.getDeployDirectory().toPath().resolve("paths");
    HashMapLoader mapLoader = new HashMapLoader();



    // TODO: construct TrajectoryCommand objects for Trajectory objects loaded from
    // JSON files
    // The data loaded from files is contained in mapLoader.trajectoryMap. Each
    // element's key
    // contains the name of a JSON file and its value is the Trajectory object
    // constructed from
    // the JSON data.

    // ------------------------------
    // END PathWeaver Data
    // ------------------------------
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.botState.isRedAlliance = 
      (DriverStation.Alliance.Red == DriverStation.getAlliance());
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // Reset all encoders to 0
    m_robotContainer.driveBaseSubsystem.resetEncoders();
    m_robotContainer.dashboardSubsystem.initialize(m_robotContainer);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.driveBaseSubsystem.setMotorSafetyEnabled(true);
    m_robotContainer.driveBaseSubsystem.setMotorSafetyEnabled(true);
    m_robotContainer.dashboardSubsystem.initialize(m_robotContainer);
    
    
    m_robotContainer.driveBaseSubsystem.LimitMotors(m_robotContainer.botState.currentLimitingIsEnabled);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}