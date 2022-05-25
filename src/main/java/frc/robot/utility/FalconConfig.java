// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

/** 
 * A class that encapsulates closed loop (PID) controller gains and parameters for a CTRE Falcon 
 * motor
 */
public class FalconConfig {
  /** Closed-loop gains */
  private PIDGains m_PIDGains;
  /** Integral zone (sensor units) */
  public int m_iZone;
  /** Maximum Peak output */
  public double m_peakOutput;

  /** Creates uninitialized default Gains */
  public FalconConfig() {
    m_PIDGains = new PIDGains();
    m_peakOutput = 0.0;
    m_iZone = 0;
  }

  /**
   * Creates an instance of the object with given parameter values
   * 
   * @param _enabled    True if the controller is enabled; else false to disable it
   * @param _kP         Proportional gain
   * @param _kI         Integral gain
   * @param _kD         Derivative gain
   * @param _kF         Feed-forward gain
   * @param _izone      Integral zone (i.e. integral deadband)
   * @param _peakOutput Maximum peak output
   */
  public FalconConfig(PIDGains gains, int iZone, double peakOutput) {
    m_PIDGains = gains;
    m_iZone = iZone;
    m_peakOutput = peakOutput;
  }

  /**
   * Creates an instance of the object using values copied from another object
   * @param other  Object to copy values from
   */
  public FalconConfig(FalconConfig other) {
    m_PIDGains = other.m_PIDGains;
    m_iZone = other.m_iZone;
    m_peakOutput = other.m_peakOutput;
  }

}
