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
import java.util.Arrays;
import java.util.Map;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.RobotContainer;
import frc.robot.subsystems.driveBase.WestCoastDriveTrain;
import frc.robot.subsystems.runtimeState.BotStateSubsystem;
import frc.robot.subsystems.runtimeState.BotStateSubsystem.RobotDirection;
import frc.robot.utility.PathweaverTrajectoryStore;

/**
 * A class supplying a ShuffleBoard tab containing the primary widgets visible on the drive station
 */
public class ShowtimeDashboardTab extends Object implements IDashboardTab {
  /** The Shuffleboard tab to display in */
  private ShuffleboardTab m_tab;
  /** Handle to current robot values */
  private BotStateSubsystem m_botState;
  /** Chooser used to select the active autonomous routine */
  private SendableChooser<String> m_autoRoutineChooser;
  /** Name of the selected auto routine */
  private String m_selectedAutoRoutineName;
  /** Chooser used to enable/disable limits */
  private NetworkTableEntry m_currentLimitTableEntry;
  /** Factory used to create Trajectories from Pathweaver data */
  private PathweaverTrajectoryStore m_pathweaverFactory;
    /** View of the field */
  private final Field2d m_field2d = new Field2d();

  /** Drive base subsystem handle used to obtain the present robot pose */
  private WestCoastDriveTrain m_driveBaseSubsystem;

  /**
   * Creates an instance of the tab
   * 
   * @param driveTrainSubsystem Drive Train subsystem to operate on
   */
  ShowtimeDashboardTab(RobotContainer botContainer) {
    m_botState = botContainer.botState;
    m_driveBaseSubsystem = botContainer.driveBaseSubsystem;
    m_pathweaverFactory = botContainer.trajectoryStore;
  }

  /**
   * Create and initialize dashboard widgets
   * 
   * NOTE: Built-in widgets are described here:
   * https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj/shuffleboard/BuiltInWidgets.html
   */
  @Override
  public void initialize(RobotContainer botContainer) {
    m_tab = Shuffleboard.getTab("Showtime");
    m_botState = botContainer.botState;
    boolean addedAllWidgets = true;

    // Create an auto chooser with entries for each Pathweaver Trajectory.
    m_autoRoutineChooser = new SendableChooser<String>();
    String autoNames[] = botContainer.trajectoryStore.getTrajectoryNames();
    Arrays.sort(autoNames); // Sort auto names into ascending lexical order
    for (int idx = 0; idx < autoNames.length; ++idx) {
      String name = autoNames[idx];
      m_autoRoutineChooser.addOption(name, name);
      // Set the default option to the first auto routine
      if (idx == 0) {
        m_autoRoutineChooser.setDefaultOption(name, name);
      }
    }

    // Add runtime widgets
    ShuffleboardLayout runtimeLayout = m_tab.getLayout("Runtime", BuiltInLayouts.kGrid)
        .withSize(11, 2)
        .withProperties(
            Map.of("Label position", "TOP", "Number of columns", 5, "Number of rows", 1));

    // Add a widget to indicate the current alliance
    try {
      runtimeLayout.addBoolean("Alliance", m_botState::isRedAlliance)
        .withProperties(Map.of("Color when true", "red", "Color when false", "blue"))
        .withPosition(0, 0);
    } catch (IllegalArgumentException e) {
      DriverStation.reportError("Failed to add Alliance widget", false);
      addedAllWidgets = false;
    }

    // Add autonomous routine chooser widget
    try {
      runtimeLayout.add("Auto Routine", m_autoRoutineChooser)
        .withPosition(1, 0);
    } catch (IllegalArgumentException e) {
      DriverStation.reportError("Failed to add auto chooser", false);
      addedAllWidgets = false;
    }

    // Add a widget to show the present direction
    try {
      runtimeLayout.addString("Direction", () -> {
        return (m_botState.getDriveDirection() == RobotDirection.Forward) ? "Normal" : "Reversed";
      }).withWidget(BuiltInWidgets.kTextView)
        .withPosition(2, 0);
    } catch (IllegalArgumentException e) {
      DriverStation.reportError("Failed to add motor current limit switch", false);
      addedAllWidgets = false;
    }

    // Add a widget to show manual mode status
    try {
      runtimeLayout.addBoolean("Manual Mode", m_botState::manualControlIsEnabled)
        .withProperties(Map.of("Color when true", "yellow", "Color when false", "#b3b3b3"))
        .withPosition(3, 0);

    } catch (IllegalArgumentException e) {
      DriverStation.reportError("Failed to add other switch widget", false);
      addedAllWidgets = false;
    }

    // Add a switch to enable motor current limiting
    try {
      m_currentLimitTableEntry = runtimeLayout.addPersistent("Motor Current Limit", false)
          .withWidget(BuiltInWidgets.kToggleSwitch)
          .withPosition(4, 0)
          .getEntry();
    } catch (IllegalArgumentException e) {
      DriverStation.reportError("Failed to add motor current limit switch", false);
      addedAllWidgets = false;
    }

    try {
      m_tab.add("Field", m_field2d)
           .withSize(11, 6);
    } catch (IllegalArgumentException e) {
      DriverStation.reportError("Failed to add field2d widget", false);
      addedAllWidgets = false;
    }

    // Display an error on failure to add all widgets
    if (!addedAllWidgets) {
      String msg = "Failed to initialize Showtime dashboard widgets!  Please Restart shuffleboard.";
      DriverStation.reportError(msg, false);
    }
  }

  /**
   * Service dashboard tab widgets
   */
  @Override
  public void update() {
    // Get the name of the selected auto routine.  If it has changed, send the corresponding
    // trajectory (if there is one) to be displayed on the dashboard.
    String selectedAutoName = getSelectedAutoName();
    if (selectedAutoName != m_selectedAutoRoutineName) {
      m_selectedAutoRoutineName = selectedAutoName;
      Trajectory traj = m_pathweaverFactory.getTrajectory(selectedAutoName);
      if (null != traj) {
        m_field2d.getObject("traj").setTrajectory(traj);
      } 
      else {
        // If the auto routine has no associated trajectory, clear the present Trajectory
        Trajectory emptyTrajectory = new Trajectory();
        m_field2d.getObject("traj").setTrajectory(emptyTrajectory);
      }
    }

    // Update the robot pose on the dashboard
    m_field2d.setRobotPose(m_driveBaseSubsystem.getPose());

    // Update motor limiting enablement
    m_botState.setCurrentLimitEnabled(m_currentLimitTableEntry.getBoolean(false));
  }

  /**
   * Returns the name of the presently selected Auto routine
   * 
   * @return A String containing the name of the presently selected Auto routine
   */
  public String getSelectedAutoName() {
    return m_selectedAutoRoutineName;
  }

}
