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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.autonomous.*;
import frc.robot.commands.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.runtimeState.BotStateSubsystem;
import frc.robot.subsystems.runtimeState.BotStateSubsystem.RobotDirection;
import frc.robot.subsystems.ShooterSubsystem;

// TODO: DashboardSubsystem is ugly.  It needs to be cleaned up and refactored into submodules

//import frc.robot.commands.shooter.fireBallSubcommands.FeedFlywheel;
//import frc.robot.commands.shooter.loadBallSubcommands.BackOffFlywheel;
public class DashboardSubsystem extends SubsystemBase {

  public SendableChooser<Command> m_chooser;
  public SendableChooser<String> m_stealthMode;
  public SendableChooser<String> m_NoLimits;

  private BotStateSubsystem m_botState;
  private ShooterSubsystem m_shooterSubsystem;

  /** Creates a new Dashboard. */
  public DashboardSubsystem(RobotContainer botContainer) {
    m_botState = botContainer.botState;
    m_shooterSubsystem = botContainer.shooterSubsystem;
    m_chooser = new SendableChooser<>();
    m_stealthMode = new SendableChooser<>();
    m_NoLimits = new SendableChooser<>();

    SmartDashboard.putData(m_chooser);
    m_NoLimits.setDefaultOption("Limits ON", "LimitsOn");
    m_NoLimits.addOption("Limits OFF", "LimitsOff");
    SmartDashboard.putData(m_NoLimits);
    m_stealthMode.setDefaultOption("Stealth OFF", "Normal");
    m_stealthMode.addOption("Stealth On", "Stealthy");
    SmartDashboard.putData(m_stealthMode);

    // Set up the autonomous chooser
    m_chooser.setDefaultOption("High Auto", new AutoHighGoal(botContainer));
    m_chooser.addOption("Simple Auto", new AutoDrive(botContainer));
    m_chooser.addOption("Red 1", new AutoRedOne());
    m_chooser.addOption("Red 2", new AutoRedTwo());
    m_chooser.addOption("Red 3", new AutoRedThree());
    m_chooser.addOption("Blue 1", new AutoBlueOne());
    m_chooser.addOption("Blue 2", new AutoBlueTwo());
    m_chooser.addOption("Blue 3", new AutoBlueThree());

    // SmartDashboard.putData("Fire Ball", new
    // LoadAndFire(Shooter.FlywheelSpeed.Autonomous, RobotContainer.m_Shooter,
    // null));
    // SmartDashboard.putData("DriveStage 0", new
    // DriveStage0(RobotContainer.m_DriveBase, RobotContainer.m_Intake));

    // SmartDashboard.putData("Fire Ball", new FireBall(RobotContainer.m_Shooter));
    // SmartDashboard.putData("Intake Ball", new
    // IntakeBall(RobotContainer.m_Intake));
    // SmartDashboard.putData("BackOffFlywheel", new
    // BackOffFlywheel(RobotContainer.m_Shooter));
    // SmartDashboard.putData("Back Off Flywheel", new
    // BackOffFlywheel(RobotContainer.m_Shooter));

    // SmartDashboard.putData("Feed Ball", new
    // FeedFlywheel(RobotContainer.m_Shooter, null));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // We should look at Shuffleboard as our smartdashboard.
    // https://docs.wpilib.org/en/stable/docs/software/dashboards/shuffleboard/getting-started/shuffleboard-tour.html

    // SmartDashboard.putString("Auto chooser",
    // RobotContainer.m_chooser.getSelected().getName());
    SmartDashboard.putBoolean("Drive Direction",
        m_botState.DriveDirection == RobotDirection.Forward);
    SmartDashboard.putBoolean("Manual Mode", m_botState.ManualControl);
    SmartDashboard.putBoolean("Alliance", m_botState.isRedAlliance);
    SmartDashboard.putNumber("Flywheel", m_shooterSubsystem.getShooterRPM());
    SmartDashboard.putBoolean("At Flywheel speed", m_shooterSubsystem.IndexAtLocation());
    // SmartDashboard.putNumber("Indexer",
    // RobotContainer.m_Shooter.getIndexPosition());
    // SmartDashboard.putBoolean("First Ball",
    // RobotContainer.m_Shooter.upperBallIsPresent());
    // SmartDashboard.putBoolean("Second Ball",
    // RobotContainer.m_Shooter.lowerBallIsPresent());
    // SmartDashboard.putBoolean("Ball Against Flywheel",
    // RobotContainer.m_Shooter.getBallLimitSensor());
    // SmartDashboard.putNumber("Intake Arm Position",
    // RobotContainer.m_Intake.getIntakeArmPositionRaw());
    // SmartDashboard.putNumber("Climber Position",
    // RobotContainer.m_Climber.getClimberPosition());
    // SmartDashboard.putNumber("tRex Position",
    // RobotContainer.m_tRex.getArmPositionRaw());

    // SmartDashboard.putNumber("leftDrive",
    // RobotContainer.m_DriveBase.leftMaster.getSelectedSensorPosition());
    // SmartDashboard.putNumber("rightDrive",
    // RobotContainer.m_DriveBase.rightMaster.getSelectedSensorPosition());

    // PushMotorTemps();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }

  // Motor Temp records
  // private void PushMotorTemps(){
  // SmartDashboard.putNumber("Left Drive Temp",
  // RobotContainer.m_DriveBase.leftMaster.getTemperature());
  // SmartDashboard.putNumber("Left Drive Slave Temp",
  // RobotContainer.m_DriveBase.leftSlave.getTemperature());
  // SmartDashboard.putNumber("Right Drive Temp",
  // RobotContainer.m_DriveBase.rightMaster.getTemperature());
  // SmartDashboard.putNumber("Right Drive Slave Temp",
  // RobotContainer.m_DriveBase.rightSlave.getTemperature());
  // SmartDashboard.putNumber("Climb Master Temp",
  // RobotContainer.m_Climber.ClimbMaster.getTemperature());
  // SmartDashboard.putNumber("Climb Slave Temp",
  // RobotContainer.m_Climber.ClimbSlave.getTemperature());
  // SmartDashboard.putNumber("Intake Actuator Temp",
  // RobotContainer.m_Intake.m_armMotor.getTemperature());
  // SmartDashboard.putNumber("tRex Master Temp",
  // RobotContainer.m_tRex.m_tRexMaster.getTemperature());
  // SmartDashboard.putNumber("tRex Slave Temp",
  // RobotContainer.m_tRex.m_tRexSlave.getTemperature());

  // }
}
