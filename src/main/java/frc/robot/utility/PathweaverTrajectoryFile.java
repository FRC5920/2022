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

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.io.IOException;
import java.nio.file.InvalidPathException;
import java.util.function.Consumer;

import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.Trajectory;

///////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * A utility class used to describe a Trajectory loaded from a Pathweaver JSON file
 */
public class PathweaverTrajectoryFile {

  /** Path of the file being described */
  private Path m_filePath;
  /** Trajectory loaded from the file */
  private Trajectory m_trajectory;

  /** Creates an instance of the object */
  public PathweaverTrajectoryFile(Path filePath, Trajectory trajectory) {
    m_filePath = filePath;
    m_trajectory = trajectory;
  }

  /** Returns the Trajectory loaded from the file */
  public Trajectory trajectory() {
    return m_trajectory;
  }

  /** Returns the full path of the Pathweaver JSON file associated with the object */
  public Path filePath() {
    return m_filePath;
  }

  /** Returns the base name of the Pathweaver JSON file associated with the object */
  public String fileName() {
    return m_filePath.toFile().getName();
  }

  /** Returns a 'friendly' name composed of the Pathweaver JSON file name up to the first '.' */
  public String friendlyName() {
    String fileName = fileName();
    int firstDot = fileName.indexOf(".");
    // If no dot is found in the file name, return the whole string.
    // Otherwise, return everything preceding the first dot
    return (firstDot < 0) ? fileName : fileName.substring(0, firstDot);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Iterates over files in a given directory and loads Trajectory objects from them
   * 
   * @param jsonDir      Directory containing JSON files to be loaded as Trajectory objects
   * 
   * @param loadHandler  Function called when a JSON file is loaded successfully. The argument
   *                     passed to this handler function contains the Trajectory loaded from the
   *                     file.
   * @param errorHandler Function called when a file fails to load
   * 
   * @remarks This routine iterates over each file in jsonDir and attempts to load a Trajectory from
   *          it. Files are naiively assumed to be in Pathweaver JSON format. If a Trajectory is
   *          loaded successfully, a TrajectoryFileInfo object is passed to the loadHandler. On
   *          failure to load a Trajectory, a string containing an error message is passed to
   *          errorHandler.
   * 
   * @example
   * 
   *          Given the following JSON files in the directory /home/lvuser/deploy/paths:
   *          PathA.wpilib.json PathB.wpilib.json PathZ.wpilib.json DontGoThere.wpilib.txt (not a
   *          valid JSON file)
   * 
   *          The following calls will be made to loadHandler: loadHandler(
   *          TrajectoryFileInfo("PathA", <Trajectory built from PathA.wpilib.json>) ) loadHandler(
   *          TrajectoryFileInfo("PathB", <Trajectory built from PathA.wpilib.json>) ) loadHandler(
   *          TrajectoryFileInfo("PathZ", <Trajectory built from PathA.wpilib.json>) )
   *          errorHandler("Error loading Pathweaver trajectory from DontGoThere.wpilib.txt")
   * 
   * @return The number of Trajectories loaded successfully from files
   * 
   * @throws InvalidPathException if jsonDir does not exist or is not a directory
   */
  static public int loadJSONFiles(Path jsonDir, Consumer<PathweaverTrajectoryFile> loadHandler,
      Consumer<String> errorHandler) {
    int count = 0;

    // Throw an exception if the target directory is invalid
    validateJSONDir(jsonDir);

    // Iterate over files in jsonDir
    for (File f : jsonDir.toFile().listFiles()) {
      Path filePath = Paths.get(f.getAbsolutePath());

      try {
        Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(filePath);
        PathweaverTrajectoryFile info = new PathweaverTrajectoryFile(filePath, trajectory);
        loadHandler.accept(info);
        ++count;
      } catch (IOException e) {
        errorHandler.accept(String.format("Error loading Pathweaver trajectory from %s\n%s",
            filePath.toString(), e.toString()));
      }
    }

    return count;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Throws InvalidPathException if a given directory does not exist or is not a directory */
  private static void validateJSONDir(Path jsonDir) {
    File dir = jsonDir.toFile();
    if (!dir.exists()) {
      throw new InvalidPathException(jsonDir.toString(), String.format("Directory does not exist"));
    }
    if (!dir.isDirectory()) {
      throw new InvalidPathException(jsonDir.toString(), String.format("Path is not a directory"));
    }
  }
}