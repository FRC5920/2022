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

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.trajectory.Trajectory;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
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
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow and SmartDashboard
   * integrated updating.
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
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // Reset all encoders to 0
    m_robotContainer.driveBaseSubsystem.resetEncoders();
    m_robotContainer.driveBaseSubsystem.zeroHeading();

    // Schedule the present selected autonomous command
    String selectedAutoName = m_robotContainer.dashboardSubsystem.getSelectedAutoRoutine();
    Trajectory trajectory = m_robotContainer.pathweaverFactory.getTrajectory(selectedAutoName);
    m_autonomousCommand = m_robotContainer.pathweaverFactory.createPathweaverCommand(trajectory,
        m_robotContainer.driveBaseSubsystem);

    // Display the current Trajectory in the dashboard field view
    m_robotContainer.dashboardSubsystem.setCurrentTrajectory(trajectory);
    m_autonomousCommand.schedule();
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

    //m_robotContainer.driveBaseSubsystem
    //    .limitMotors(m_robotContainer.botState.getCurrentLimitEnabled());
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