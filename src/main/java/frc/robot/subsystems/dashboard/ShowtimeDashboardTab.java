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

package frc.robot.subsystems.dashboard;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import frc.robot.RobotContainer;
import frc.robot.commands.autonomous.*;
import frc.robot.subsystems.runtimeState.BotStateSubsystem;
import frc.robot.subsystems.runtimeState.BotStateSubsystem.RobotDirection;


/** 
 * A class supplying a ShuffleBoard tab containing the primary widgets visible on the drive station
 */
public class ShowtimeDashboardTab extends Object implements IDashboardTab {
  /** The Shuffleboard tab to display in */
  private ShuffleboardTab m_tab;
  /** Handle to current robot values */
  private BotStateSubsystem m_botState;
  /** Chooser used to select the active autonomous routine */
  public SendableChooser<Command> m_autoRoutineChooser;

  /** Chooser used to enable/disable stealth mode */
  public SendableChooser<String> m_stealthMode;
  /** Chooser used to enable/disable limits */
  public SendableChooser<String> m_noLimits;

  /**
   * Creates an instance of the tab
   * 
   * @param driveTrainSubsystem Drive Train subsystem to operate on
   */
  ShowtimeDashboardTab(RobotContainer botContainer) {
    m_botState = botContainer.botState;
  }

  /**
   * Create and initialize dashboard widgets
   */
  @Override
  public void initialize(RobotContainer botContainer) {
    m_tab = Shuffleboard.getTab("Showtime");
    m_botState = botContainer.botState;

    m_autoRoutineChooser = new SendableChooser<>();
    m_autoRoutineChooser.addOption("Simple Auto", new Auto(botContainer.driveBaseSubsystem));
    m_autoRoutineChooser.addOption("Red 1", new AutoRedOne());
    m_autoRoutineChooser.addOption("Red 2", new AutoRedTwo());
    m_autoRoutineChooser.addOption("Red 3", new AutoRedThree());
    m_autoRoutineChooser.addOption("Blue 1", new AutoBlueOne());
    m_autoRoutineChooser.addOption("Blue 2", new AutoBlueTwo());
    m_autoRoutineChooser.addOption("Blue 3", new AutoBlueThree());

    m_stealthMode = new SendableChooser<>();
    m_stealthMode.setDefaultOption("Stealth OFF", "Normal");
    m_stealthMode.addOption("Stealth On", "Stealthy");

    m_noLimits = new SendableChooser<>();
    m_noLimits.setDefaultOption("Limits ON", "LimitsOn");
    m_noLimits.addOption("Limits OFF", "LimitsOff");

    try {
      m_tab.add("Auto Routine", m_autoRoutineChooser);
      m_tab.add("No Limits", m_noLimits);
      m_tab.add("Stealth Mode", m_stealthMode);

      m_tab.addBoolean("Drive Direction", () -> {return m_botState.DriveDirection == RobotDirection.Forward;});
      m_tab.addBoolean("Manual Mode", () -> {return m_botState.ManualControl;});
      m_tab.addBoolean("Alliance", () -> {return m_botState.isRedAlliance;});
    } catch (Exception e) {
      ;
    }
  }

  /**
   * Service dashboard tab widgets
   */
  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("Drive Direction",
    //     m_botState.DriveDirection == RobotDirection.Forward);
    // SmartDashboard.putBoolean("Manual Mode", m_botState.ManualControl);
    // SmartDashboard.putBoolean("Alliance", m_botState.isRedAlliance);

    m_botState.StealthMode = ("Stealthy" == m_stealthMode.getSelected());
    m_botState.currentLimitingIsEnabled = (m_noLimits.getSelected() == "LimitsOn");
  }

}
