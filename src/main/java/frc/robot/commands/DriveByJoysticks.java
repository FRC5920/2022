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

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.driveBase.DriveBaseSubsystem;
import frc.robot.subsystems.joystick.JoystickSubsystem;
import frc.robot.subsystems.runtimeState.BotStateSubsystem;
import frc.robot.subsystems.runtimeState.BotStateSubsystem.RobotDirection;;

public class DriveByJoysticks extends CommandBase {
  private BotStateSubsystem m_botState;
  private JoystickSubsystem m_joystickSubsystem;
  private DriveBaseSubsystem m_driveBaseSubsystem;
  private double m_drivePowerModifer = 1;

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Creates an instance of the command
   * 
   * @param muzzleVelocity Speed to fire the ball
   * @param botContainer       Object providing access to robot subsystems
   * @param fireButton     Joystick button used to fire in manual tele-operated
   *                       mode
   */
  public DriveByJoysticks(RobotContainer botContainer) {
    m_botState = botContainer.botState;
    m_driveBaseSubsystem = botContainer.driveBaseSubsystem;
    m_joystickSubsystem = botContainer.joystickSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_botState, m_driveBaseSubsystem, m_joystickSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: move dashboard code in DriveByJoystick into the DashboardSubsystem
    if (m_joystickSubsystem.driverController.bumpRight.get()) {
      m_drivePowerModifer = Constants.MotorScaler.DriveMidLimit;
      SmartDashboard.putString("Speed", "Medium");
    } else {
      if (m_joystickSubsystem.driverController.bumpLeft.get()) {
        m_drivePowerModifer = Constants.MotorScaler.DriveSlowLimit;
        SmartDashboard.putString("Speed", "Slow");
      } else {
        m_drivePowerModifer = Constants.MotorScaler.DriveStandardLimit;
        SmartDashboard.putString("Speed", "Normal");
      }
    }


    if (m_botState.DriveDirection == RobotDirection.Forward) {
      m_driveBaseSubsystem.tankDrive(m_drivePowerModifer * m_joystickSubsystem.driverController.rightStickY(),
          m_drivePowerModifer * m_joystickSubsystem.driverController.leftStickY());

    } else {
      m_driveBaseSubsystem.tankDrive(-1 * m_drivePowerModifer * m_joystickSubsystem.driverController.leftStickY(),
          -1 * m_drivePowerModifer * m_joystickSubsystem.driverController.rightStickY());
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveBaseSubsystem.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
