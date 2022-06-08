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

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.subsystems.driveBase.WestCoastDriveTrain;

/**
 * A class supplying a Shuffleboard tab for configuring drive train parameters
 */
public class DriveTrainDashboardTab implements IDashboardTab {
  private static final int kLeft = 0;
  private static final int kRight = 1;

  /** Drive train to operate on */
  private WestCoastDriveTrain m_driveTrainSubsystem;
  /** The Shuffleboard tab to display in */
  private ShuffleboardTab m_tab;
  
  /** The current X position (meters) */
  NetworkTableEntry m_poseX;
  /** The current Y position (meters) */
  NetworkTableEntry m_poseY;
  /** The current rotation (degrees) */
  NetworkTableEntry m_poseRotation;

  /////////////////////////////////////////////////////////////
  // Telemetry for left and right sides of the Drive Train
  /////////////////////////////////////////////////////////////
  /** NetworkTable entry used to display distance traveled in meters */
  NetworkTableEntry m_distanceMeters[];
  /** NetworkTable entry used to display velocity in meters/sec */
  NetworkTableEntry m_velocityMetersPerSec[];
  /** NetworkTable entry used to display velocity error */
  NetworkTableEntry m_velocityError[];

  /**
   * Creates an instance of the tab
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
    ShuffleboardLayout poseLayout = m_tab.getLayout("Pose", BuiltInLayouts.kGrid)
                .withSize(4, 1)
                .withPosition(0, 0);
    
    m_poseX = poseLayout.add("X (m)", 0.0)
      .withSize(2, 2)
      .withPosition(0, 0)
      .getEntry();

    m_poseY = poseLayout.add("Y (m)", 0.0)
      .withSize(2, 2)
      .withPosition(1, 0)
      .getEntry();

    m_poseRotation = poseLayout.add("Rot (deg)", 0.0)
      .withSize(2, 2)
      .withPosition(2, 0)
      .getEntry();

    String sideName[] = {"Left", "Right"};
    m_distanceMeters = new NetworkTableEntry[2];
    m_velocityMetersPerSec = new NetworkTableEntry[2];
    m_velocityError = new NetworkTableEntry[2];
    for (int side = kLeft; side <= kRight; ++side) {
      ShuffleboardLayout telemetryLayout = m_tab.getLayout(sideName[side] + " Telemetry", BuiltInLayouts.kGrid)
        .withSize(2, 2)
        .withPosition(side * 2, 1)
        .withProperties(Map.of("Label position", "LEFT"));
      m_distanceMeters[side] = telemetryLayout.add("Distance", 0.0)
        .withSize(2,1)
        .withPosition(0, 0)
        .getEntry();
      m_velocityMetersPerSec[side] = telemetryLayout.add("Velocity", 0.0)
        .withSize(2,1)
        .withPosition(0, 1)
        .getEntry();
      m_velocityError[side] = telemetryLayout.add("Velocity Error", 0.0)
        .withSize(2,1)
        .withPosition(0, 2)
        .getEntry();
    }

    // Set up a gain tuner layout for each side of the drive train
    ShuffleboardLayout leftTuner = m_tab.getLayout("Left Drivebase", BuiltInLayouts.kList)
        .withProperties(Map.of("Label position", "TOP"));
    try {
      leftTuner.add("Left PID Control", m_driveTrainSubsystem.getSendableGains()[0]);
      leftTuner.addNumber("Left PID Error", () -> { return m_driveTrainSubsystem.getVelocitySensorError()[0]; });
    } catch (Exception e) {}

    ShuffleboardLayout rightTuner = m_tab.getLayout("Right Drivebase", BuiltInLayouts.kList)
        .withProperties(Map.of("Label position", "TOP"));
    try {
      rightTuner.add("Right PID Control", m_driveTrainSubsystem.getSendableGains()[1]);
      rightTuner.addNumber("Right PID Error", () -> { return m_driveTrainSubsystem.getVelocitySensorError()[1]; });
    } catch (Exception e) {}
  }

  /**
   * Service dashboard tab widgets
   */
  @Override
  public void update() {
    double distanceSI[] = m_driveTrainSubsystem.getSensorDistanceMeters();
    //double velocitySI[] = m_driveTrainSubsystem.getEncoderVelocitySI();
    double velocityError[] = m_driveTrainSubsystem.getVelocitySensorError();
    DifferentialDriveWheelSpeeds speeds = m_driveTrainSubsystem.getWheelSpeeds();
    for (int side = kLeft; side <= kRight; ++side) {
      m_distanceMeters[side].setDouble(distanceSI[side]);
      m_velocityMetersPerSec[side].setDouble(side == kLeft ? speeds.leftMetersPerSecond : speeds.rightMetersPerSecond);
      m_velocityError[side].setDouble(velocityError[side]);
    }

  }
}
