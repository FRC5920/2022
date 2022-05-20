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

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import frc.robot.RobotContainer;
import frc.robot.subsystems.driveBase.WestCoastDriveTrain;

/**
 * A class supplying a Shuffleboard tab for configuring drive train parameters
 */
public class DriveTrainDashboardTab implements IDashboardTab {
  /** The Shuffleboard tab to display in */
  private ShuffleboardTab m_tab;
  /** Drive train to operate on */
  private WestCoastDriveTrain m_driveTrainSubsystem;

  /**
   * Creates an instance of the tab
   * 
   * @param driveTrainSubsystem Drive Train subsystem to operate on
   */
  DriveTrainDashboardTab(WestCoastDriveTrain driveTrainSubsystem) {
    m_driveTrainSubsystem = driveTrainSubsystem;
  }

  /**
   * Create and initialize dashboard widgets
   */
  @Override
  public void initialize(RobotContainer botContainer) {
    m_tab = Shuffleboard.getTab("Drive Train");

    // Set up a gain tuner layout for each side of the drive train
    ShuffleboardLayout leftTuner = m_tab.getLayout("Left Drivebase", BuiltInLayouts.kList)
        .withProperties(Map.of("Label position", "TOP"));
    try {
      leftTuner.add("Left PID Control", m_driveTrainSubsystem.getLeftGains());
      leftTuner.addNumber("Left PID Error", () -> { return m_driveTrainSubsystem.getLeftMotorError(); });
    } catch (Exception e) {}

    ShuffleboardLayout rightTuner = m_tab.getLayout("Right Drivebase", BuiltInLayouts.kList)
        .withProperties(Map.of("Label position", "TOP"));
    try {
      rightTuner.add("Right PID Control", m_driveTrainSubsystem.getRightGains());
      rightTuner.addNumber("Right PID Error", () -> { return m_driveTrainSubsystem.getRightMotorError(); });
    } catch (Exception e) {}
  }

  /**
   * Service dashboard tab widgets
   */
  @Override
  public void periodic() {
    // Report motor temperatures
    // SmartDashboard.putNumber("Left Drive Temp",
    // m_DriveBase.leftMaster.getLeftMotorTemperature());
    // SmartDashboard.putNumber("Right Drive Temp",
    // m_DriveBase.rightMaster.getRightMotorTemperature());

    // Update Drive Base motor gains if drive base gain tuner values have been modified
    // if (m_gainTunerLeft.process()) {
    // m_driveTrainSubsystem.updateLeftMotorGains();
    // }
    // if (m_gainTunerRight.process()) {
    // m_driveTrainSubsystem.updateRightMotorGains();
    // }
  }
}
