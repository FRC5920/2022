// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/** Add your docs here. */
public class SendableGains implements Sendable {
  /** true after gains have been modified; false after setUnmodified() is called */
  private boolean m_gainsModified;
  /** PIDGains to operate on */
  private PIDGains m_gains;

  /**
   * Constructs a SendableGains that operates on a given PIDGains object
   * 
   * @param gains PIDGains to operate on
   */
  public SendableGains(PIDGains gains) {
    m_gainsModified = false;
    m_gains = gains;
  }

  /** Returns true if the gains are enabled */
  public boolean getEnabled() {
    return m_gains.enabled();
  }

  /** Returns the proportional gain */
  public double getkP() {
    return m_gains.kP();
  }

  /** Returns the integral gain */
  public double getkI() {
    return m_gains.kI();
  }

  /** Returns the derivative gain */
  public double getkD() {
    return m_gains.kD();
  }

  /** Returns the feed-forward gain */
  public double getkF() {
    return m_gains.kF();
  }

  /**
   * Sets whether PID control is enabled
   * @param value  true to enable PID control; else false to disable PID control
   */
  public void setEnabled(boolean value) {
    m_gains.enabled(value);
    m_gainsModified = true;
  }

  /**
   * Sets the proportional gain and marks gains as modified
   * @param value  Gain value to assign
   */
  public void setkP(double value) {
    m_gains.kP(value);
    m_gainsModified = true;
  }

  /**
   * Sets the integral gain and marks gains as modified
   * @param value  Gain value to assign
   */
  public void setkI(double value) {
    m_gains.kI(value);
    m_gainsModified = true;
  }

  /**
   * Sets the derivative gain and marks gains as modified
   * @param value  Gain value to assign
   */
  public void setkD(double value) {
    m_gains.kD(value);
    m_gainsModified = true;
  }

  /**
   * Sets the feed-forward gain and marks gains as modified
   * @param value  Gain value to assign
   */
  public void setkF(double value) {
    m_gains.kF(value);
    m_gainsModified = true;
  }

  /**
   * @return true if values in the object have been modified since isModified()
   *         was last called.
   * 
   * @postcondition Upon return, the object is no longer marked as modified
   */
  public boolean isModified() {
    boolean result = m_gainsModified;
    m_gainsModified = false;
    return result;
  }

  /**
   * Initializes the object as a Sendable
   * 
   * @param builder sendable builder
   */
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("PIDController");
    builder.addDoubleProperty("p", this::getkP, this::setkP);
    builder.addDoubleProperty("i", this::getkI, this::setkI);
    builder.addDoubleProperty("d", this::getkD, this::setkD);
    builder.addDoubleProperty("f", this::getkF, this::setkF);
  }

}
