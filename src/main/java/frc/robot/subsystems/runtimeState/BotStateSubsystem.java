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

package frc.robot.subsystems.runtimeState;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BotStateSubsystem extends SubsystemBase {

  /////////////////////////////////////////////////////////////////////////////
  /** Directions used when driving the robot */
  public enum RobotDirection {
    /** Forward commands drive the robot toward its front (normal) */
    Forward,
    
    /** Forward commands drive the robot toward its rear */
    Reverse;

    /** Get the human-readable name of the direction */
    @Override
    public String toString() {
      return this.name();
    }
  };

  /** true when manual control is active; else false */
  private boolean m_manualControl = false;
  /** The present direction modifier for manual control */
  private RobotDirection m_driveDirection = RobotDirection.Forward;
  /** true when the robot is shooting; else false */
  private boolean m_robotIsShooting = false;
  /** true when motor current limiting is enabled; else false */
  private boolean m_currentLimitingIsEnabled = false;

  /** 
   * Creates an instance of the object
  */
  public BotStateSubsystem() 
  {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }  

  /** Gets the current manual control enablement 
   * @return true if manual control is enabled
  */
  public boolean manualControlIsEnabled() {
    return m_manualControl;
  }

  /** Enables/disables manual control 
   * @param enable  true to enable manual control; else false
  */
  public void enableManualControl(boolean enable) {
    m_manualControl = enable;
  }

  /** Gets the current drive direction */
  public RobotDirection getDriveDirection() {
    return m_driveDirection;
  }

  /**
   * Inverts the present drive direction of the bot
  */
  public void invertDriveDirection() {
    m_driveDirection = (RobotDirection.Forward == m_driveDirection) ?
                        RobotDirection.Reverse : RobotDirection.Forward;
  }

  /** Returns whether the robot is shooting or not 
   * @return true if the robot is presently shooting; else false
  */
  public boolean robotIsShooting() {
    return m_robotIsShooting;
  }

  /** Returns whether the present alliance is the Red alliance
   * @return true if the present alliance is Red; else false
   */
  public boolean isRedAlliance() {
    return DriverStation.Alliance.Red == DriverStation.getAlliance();
  }

  /** Returns whether the present alliance is the Blue alliance
   * @return true if the present alliance is Blue; else false
   */
  public boolean isBlueAlliance() {
    return DriverStation.Alliance.Blue == DriverStation.getAlliance();
  }

  /** Sets whether the  */
  /**
   * Returns true if the robot is being driven in Manual, tele-operated mode
   */
  public boolean robotIsInManualTeleOpMode() {
    return (RobotState.isEnabled() && RobotState.isTeleop() && m_manualControl);
  }

  /** Gets the enablement of motor current limiting
   * @return true if motor current limiting is enabled; else false
   */
  public boolean getCurrentLimitEnabled() {
    return m_currentLimitingIsEnabled;
  }

  /** Sets motor current limiting enablement
   * @param enabled  true to enable motor current limiting; else false to disable
   */
  public void setCurrentLimitEnabled(boolean enabled) {
    m_currentLimitingIsEnabled = enabled;
  }

}
