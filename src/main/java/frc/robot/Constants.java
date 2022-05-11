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

package frc.robot;
//package edu.wpi.first.wpilibj.examples.ramsetecommand;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import frc.robot.utility.Gains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {
	/**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
	public static final int kSlotIdx = 0;

	/**
	 * Talon FX supports multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/**
	 * Set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
	public static final int kTimeoutMs = 30;

	/* Choose so that Talon does not report sensor out of phase */
	public static boolean kSensorPhase = true;

	/**
	 * Choose based on what direction you want to be positive,
	 * this does not affect motor invert.
	 */
	public static boolean kMotorInvert = false;

	/**
	 * Gains used in Positon Closed Loop, to be adjusted accordingly
	 * Gains(kp, ki, kd, kf, izone, peak output);
	 * Gains(double _kP, double _kI, double _kD, double _kF, int _kIzone, double
	 * _kPeakOutput)
	 */
	// public static Gains kGains = new Gains(0.15, 0.0, 1.0, 0.0, 0, 1.0);
	public static Gains ktRexGains = new Gains(true, 0.025, 0.0, 0.0, 0, 0, 0.1);
	public static Gains kIntakeGains = new Gains(true, 0.025, 0.0, 0.0, 0, 0, 0.1);

	// http://www.ni.com/en-us/innovations/white-papers/06/pid-theory-explained.html
	public static double JoystickSensitivity = .3;

	public static final int[] ArmPosition = new int[] { 0, 17850, 23000 };
	public static final int[] tRexPosition = new int[] { 0, 13629, 31880, 43589, 54841, 68187 };

	public static final class BatteryBudget {
		/**
		 * Nominal peak current the battery is expected to reliably deliver (Amps)
		 * 
		 * @remarks
		 *          The FRC battery can deliver a theoretical maximum load of 300 Amps.
		 *          In practice,
		 *          however, FRC batteries generally deliver less than this,
		 *          particularly after they
		 *          have been used through numerous charge cycles. This constant should
		 *          be set to a
		 *          value that is lower by a reasonable margin than the theoretical
		 *          maximum.
		 */
		public static final double kPeakBatteryCurrentAmps = 300 * 0.8; // 20% margin below max

		/**
		 * Percent of the overall current budget that is allocated to the drive motors
		 * 
		 * @remarks
		 *          This constant sets the percentage of the peak battery current that
		 *          will be budgeted
		 *          for the drive motors. Generally speaking, the current drawn by these
		 *          motors when
		 *          the robot is moving represents the vast majority of battery loading.
		 *          This constant
		 *          sets up a maximum percentage of the overall battery loading that
		 *          will be applied to
		 *          limit motor current.
		 */
		public static final double kDriveCurrentBudgetPercent = 0.8;

		/** Maximum current available to the drive motors */
		public static final double kDriveMotorCurrentAmps = (kPeakBatteryCurrentAmps * kDriveCurrentBudgetPercent) / 4;

	}

	// For Pathfinder. These need to be updated for our bot!!!!!!!
	// ------------------------------------------------------------
	public static final class DriveConstants {
		public static final int kLeftMotor1Port = 1;
		public static final int kLeftMotor2Port = 11;
		public static final int kRightMotor1Port = 2;
		public static final int kRightMotor2Port = 12;

		public static final int[] kLeftEncoderPorts = new int[] { 1, 11 };
		public static final int[] kRightEncoderPorts = new int[] { 2, 12 };
		public static final boolean kLeftEncoderReversed = false;
		public static final boolean kRightEncoderReversed = true;

		public static final double kTrackwidthMeters = 0.69;
		public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
				kTrackwidthMeters);

		public static final int kEncoderCPR = 2048;
		public static final double kGearBox = 8.45; // Defined 2/27/2022
		public static final double kWheelDiameterMeters = 0.1018; // Measured 2/27/2022
		public static final double kEncoderDistancePerPulse = (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR
				* kGearBox;

		// These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
		// These characterization values MUST be determined either experimentally or
		// theoretically
		// for *your* robot's drive.
		// The Robot Characterization Toolsuite provides a convenient tool for obtaining
		// these
		// values for your robot.
		// https://github.com/frc6421/characterization-notes/blob/main/DataAnalyze_20210704.png
		public static final double ksVolts = 0.22;
		public static final double kvVoltSecondsPerMeter = 1.98;
		public static final double kaVoltSecondsSquaredPerMeter = 0.2;

		// Example value only - as above, this must be tuned for your drive!
		public static final double kPDriveVel = 8.5;
	}

	public static final class OIConstants {
		public static final int kDriverControllerPort = 0;
	}

	public static final class AutoConstants {
		public static final double kMaxSpeedMetersPerSecond = 3;
		public static final double kMaxAccelerationMetersPerSecondSquared = 3;

		// Reasonable baseline values for a RAMSETE follower in units of meters and
		// seconds
		public static final double kRamseteB = 2;
		public static final double kRamseteZeta = 0.7;
	}

	// This is the multiplier for the speed of the intake motor. Normal speed is 1.0
	public static final class MotorScaler {
		public static final double kIntakeActuator = 1.0;
		public static final double kTRexArmManualSpeed = 0.2; // Speed when running the T-rex arms in manual mode
		public static final double kIntakeMotorSpeed = 0.7; // Speed to run the motor on the intake arm
		public static final double kClimberSpeed = 0.75; // 1.0;
    //These are voltage limits now no longer percentage
		public static final double DriveStandardLimit = 11;
		public static final double DriveMidLimit = 6;
		public static final double DriveSlowLimit = 4;
	}

	public static final class ShooterConstants {
		// Digital input ID's of the ball index optical sensors
		public static final int BallSensorDigitalInputUpper = 2;
		public static final int BallSensorDigitalInputMiddle = 1;
		public static final int BallSensorDigitalInputLower = 0;
		/**
		 * Maximum number of seconds to run the index motor to feed a ball
		 * into the flywheel to shoot it.
		 */
		public static final double kFeedFlywheelTimeoutSeconds = 2.0;
		public static final double kChamberBallTimeoutSeconds = 2.0;
	}

	// Motor identification
	public static final class MotorID {
		public static final int leftDriveMaster = 1; // Falcon 500
		//public static final int leftDriveSlave = 11; // Falcon 500
		public static final int rightDriveMaster = 2; // Falcon 500
		//public static final int rightDriveSlave = 12; // Falcon 500
	//	public static final int tRexMaster = 4; // Falcon 500
	//	public static final int tRexSlave = 14; // Falcon 500
	//	public static final int climberMaster = 5; // Falcon 500
	//	public static final int climberSlave = 15; // Falcon 500
	//	public static final int flywheel = 16; // Falcon 500
	//	public static final int intakeMotor = 32; // SRX 775
	//	public static final int indexMotor = 20; // Falcon 500
		public static final int intakeActuator = 3; // Falcon 500
	}

}
