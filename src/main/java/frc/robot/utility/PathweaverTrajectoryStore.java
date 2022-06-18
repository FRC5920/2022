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

package frc.robot.utility;

import java.nio.file.InvalidPathException;
import java.nio.file.Path;
import java.util.HashMap;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

///////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * A class to load Trajectory objects from Pathweaver JSON files and creates Trajectory commands to
 * carry them out
 */
public class PathweaverTrajectoryStore {

  /** A map of auto routine names to Trajectory objects */
  public final HashMap<String, Trajectory> m_trajectoryMap = new HashMap<String, Trajectory>();

  /**
   * Initializes the factory with trajectories loaded from JSON files. This constructor must be
   * called from the Robot.robotInit() routine, prior to the Scheduler starting to execute because
   * it performs disk I/O that can last multiple scheduler cycles.
   * 
   * @param jsonDir Relative path of a directory containing Pathweaver JSON files
   */
  public PathweaverTrajectoryStore(Path jsonDir) {
    try {
      Path jsonDirPath = Filesystem.getDeployDirectory().toPath().resolve(jsonDir);
      PathweaverTrajectoryFile.loadJSONFiles(jsonDirPath, this::onTrajectoryFileLoaded,
          this::onTrajectoryFileFailure);
    } catch (InvalidPathException e) {
      String msg = String.format("Failed to open paths directory %s\n%s", jsonDir.toString(),
          e.getCause());
      DriverStation.reportError(msg, false);
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Gets an array containing the names of available Trajectories loaded from Pathweaver JSON files
   * 
   * @return An array containing the names of Trajectories loaded from Pathweaver JSON files
   */
  public String[] getTrajectoryNames() {
    return m_trajectoryMap.keySet().toArray(new String[m_trajectoryMap.size()]);
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Get the Trajectory that was loaded from a Pathweaver JSON file
   * 
   * @param name Name string returned from getTrajectoryNames()
   * @return Trajectory object corresponding to the given name
   */
  public Trajectory getTrajectory(String name) {
    return m_trajectoryMap.get(name);
  }

  /**
   * Method called when a Trajectory is loaded successfully from a Pathweaver JSON file
   * 
   * @param fileName   Name of the file the trajectory was loaded from
   * @param trajectory Trajectory object that was loaded
   */
  public void onTrajectoryFileLoaded(PathweaverTrajectoryFile fileInfo) {
    m_trajectoryMap.put(fileInfo.friendlyName(), fileInfo.trajectory());
  }

  /**
   * Method called on failure to load the contents of a Pathweaver JSON file
   * 
   * @param errorMessage Error message describing the failure
   */
  public void onTrajectoryFileFailure(String errorMessage) {
    DriverStation.reportError(errorMessage, false);
  }
}
