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

package frc.robot.utility.trajectoryLoader;

import java.io.File;
import java.nio.file.InvalidPathException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.io.IOException;

import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.Trajectory;

///////////////////////////////////////////////////////////////////////////////
/**
 * A utility class used to create Trajectory objects from files in a given
 * directory containing JSON trajectory files.
 * 
 * To use this class, create a TrajectoryLoader object, passing it the path
 * of a directory containing JSON files to be loaded. Then, call the
 * loadJSONFiles(), passing it an object that implements the interface
 * iTrajectoryLoaderObserver.
 */
public class TrajectoryLoader {

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Validates the object's target directory, throwing an InvalidPathException
   * if it is not valid
   */
  private static void validateJSONDir(Path jsonDir) {
    File dir = jsonDir.toFile();
    if (!dir.exists()) {
      throw new InvalidPathException(jsonDir.toString(), String.format("Directory does not exist"));
    }
    if (!dir.isDirectory()) {
      throw new InvalidPathException(jsonDir.toString(), String.format("Path is not a directory"));
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Iterate over files in a given directory and create Trajectory objects from
   * each one.
   * 
   * @param jsonDir  Directory containing JSON files to be loaded as Trajectory
   *                 objects
   * 
   * @param observer LoadObserver object used to handle the processing of JSON
   *                 files
   * 
   * @remarks
   *   This routine iterates over each JSON file in jsonDir and uses it to
   *   construct a Trajectory object. On success, the name of the file and
   *   the constructed Trajectory are passed to observer.onTrajectoryCreated().
   *   If a file can't be loaded into a Trajectory object, observer.onLoadFailed()
   *   is called with the path of the failed file.
   * 
   * @example
   *          
   *   Given the following JSON files in the directory /home/lvuser/deploy/paths:
   *      PathA.wpilib.json
   *      PathB.wpilib.json
   *      PathZ.wpilib.json
   *      DontGoThere.wpilib.txt (not a valid JSON file)
   * 
   *   The following calls will be made to an Observer object:
   *      observer.onTrajectoryCreated("PathA", <Trajectory built from PathA.wpilib.json>)
   *      observer.onTrajectoryCreated("PathB", <Trajectory built from PathA.wpilib.json>)
   *      observer.onTrajectoryCreated("PathZ", <Trajectory built from PathA.wpilib.json>)
   *      observer.onLoadFailed("DontGoThere.wpilib.txt")
   * 
   * @return The number of JSON files loaded successfully
   * 
   * @throws InvalidPathException if jsonDir does not exist or is not a directory
   */
  static public int loadJSONFiles(Path jsonDir, LoadObserver observer) {
    int count = 0;

    // Throw an exception if the target directory is invalid
    validateJSONDir(jsonDir);

    // Iterate over files in jsonDir
    for (File f : jsonDir.toFile().listFiles()) {
      Path filePath = Paths.get(f.getAbsolutePath());
      String fileName = f.getName();

      try {
        Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(filePath);
        observer.onTrajectoryCreated(fileName, trajectory);
        ++count;
      } catch (IOException e) {
        observer.onLoadFailed(filePath, e.toString());
      }
    }

    return count;
  }

}