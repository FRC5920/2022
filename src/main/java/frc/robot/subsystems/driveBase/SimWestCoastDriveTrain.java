package frc.robot.subsystems.driveBase;

import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.VecBuilder;



public class SimWestCoastDriveTrain {
  /** Moment of inertia of the simulated drivetrain about its center (Kgm^2) */
  private static final double kSimulatedMomentOfInertia = 7.5;
  
  /** Mass of the simulated drive train (kg) */
  private static final double kSimulatedMassKg = 60.0;

  /** Wheel radius of the simulated drive train (meters) */
  private static final double kSimulatedWheelRadiusMeters = (WCDriveConstants.PhysicalSI.kWheelDiameterMeters / 2.0);

  /** Track width of the simulated drive train (meters) */
  private static final double kSimulatedTrackWidthMeters = WCDriveConstants.PhysicalSI.kTrackWidthMeters;


  /** X position measurement noise standard deviation (meters) */
  private static final double XNoiseSigma = 0.001;

  /** Y position measurement noise standard deviation (meters) */
  private static final double YNoiseSigma = 0.001;

  /** Heading measurement noise standard deviation (radians) */
  private static final double HeadingNoiseSigma = 0.001;

  /** Left velocity measurement noise standard deviation (m/s) */
  private static final double LeftVelocityNoiseSigma = 0.001;

  /** Right velocity measurement noise standard deviation (m/s) */
  private static final double RightVelocityNoiseSigma = 0.001;

  /** Left position measurement noise standard deviation (m/s) */
  private static final double LeftPositionNoiseSigma = 0.001;

  /** Right position measurement noise standard deviation (m/s) */
  private static final double RightPositionNoiseSigma = 0.001;


// Create the simulation model of our drivetrain.
  DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
  DCMotor.getFalcon500(WCDriveConstants.Motors.kNumMotors),       // A motor representing the left hand side of the drive train
  WCDriveConstants.PhysicalSI.kGearRatio,                    // gear ratio on the drive train
  kSimulatedMomentOfInertia,                     // MOI of 7.5 kg m^2 (from CAD model).
  kSimulatedMassKg,                    // The mass of the robot is 60 kg.
  kSimulatedWheelRadiusMeters, // The robot uses 3" radius wheels.
  kSimulatedTrackWidthMeters,                  // The track width is 0.7112 meters.

  // The standard deviations for measurement noise
  VecBuilder.fill(XNoiseSigma, YNoiseSigma, HeadingNoiseSigma, 
                  LeftVelocityNoiseSigma, RightVelocityNoiseSigma, 
                  LeftPositionNoiseSigma, RightPositionNoiseSigma));
}
