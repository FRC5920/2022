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

import edu.wpi.first.wpilibj.Filesystem;

import frc.robot.commands.DriveByJoysticks;
import frc.robot.subsystems.driveBase.WestCoastDriveTrain;
import frc.robot.subsystems.joystick.JoystickSubsystem;
import frc.robot.subsystems.dashboard.DashboardSubsystem;
import frc.robot.subsystems.runtimeState.BotStateSubsystem;
import frc.robot.commands.autonomous.PathweaverCommandFactory;

/////////////////////////////////////////////////////////////////////////////
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // ----------------------------------
  // ROBOT SUBSYSTEMS
  // ----------------------------------
  public JoystickSubsystem joystickSubsystem;
  public BotStateSubsystem botState;
  public WestCoastDriveTrain driveBaseSubsystem;
  public DashboardSubsystem dashboardSubsystem;
  public PathweaverCommandFactory pathweaverFactory;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Load Trajectory objects from JSON files under the 'deploy/paths' directory
    // Reference: Importing a PathWeaver JSON
    // https://docs.wpilib.org/en/stable/docs/software/pathplanning/pathweaver/integrating-robot-program.html
    pathweaverFactory = new PathweaverCommandFactory(
      Filesystem.getDeployDirectory().toPath().resolve("paths/output"));
    
    // Initialize subsystems
    botState = new BotStateSubsystem();
    joystickSubsystem = new JoystickSubsystem();
    driveBaseSubsystem = new WestCoastDriveTrain();
    dashboardSubsystem = new DashboardSubsystem(this);

    // Configure the button bindings
    joystickSubsystem.configureButtonBindings(this);

    // Make the drive base be driven by Joystick commands when not processing
    // another command
    driveBaseSubsystem.setDefaultCommand(new DriveByJoysticks(this));
    dashboardSubsystem.initialize(this);
  }

}
