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

import frc.robot.commands.DriveByJoysticks;
import frc.robot.subsystems.driveBase.WestCoastDriveTrain;
import frc.robot.subsystems.joystick.JoystickSubsystem;
import frc.robot.subsystems.dashboard.DashboardSubsystem;
import frc.robot.subsystems.runtimeState.BotStateSubsystem;

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

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    joystickSubsystem = new JoystickSubsystem();
    botState = new BotStateSubsystem();
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
