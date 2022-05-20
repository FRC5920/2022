// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;


/** Add your docs here. */
public class SendableGains extends Gains implements Sendable {
  /** Flag that is true if gains have been modified via dashboard widgets */
  private boolean m_gainsModified;

  /**
   * Constructs an object using initial values given in a Gains object
   * 
   * @param val Initial values to assign to the object
   */
  public SendableGains() {
    super();
    m_gainsModified = false;
  }

  /**
   * Constructs an object using initial values given in a Gains object
   * 
   * @param val Initial values to assign to the object
   */
  public SendableGains(Gains gains) {
    super(gains);
    m_gainsModified = false;
  }

  /** Returns true if the gains are enabled */
  public boolean getEnabled() {
    return enabled;
  }

  /** Returns the proportional gain */
  public double getkP() {
    return kP;
  }

  /** Returns the integral gain */
  public double getkI() {
    return kI;
  }

  /** Returns the derivative gain */
  public double getkD() {
    return kD;
  }

  /** Returns the feed-forward gain */
  public double getkF() {
    return kF;
  }

  /** Returns the integral zone/deadband */
  public int getiZone() {
    return iZone;
  }

  /** Returns the maximum allowed controller output */
  public double getPeakOutput() {
    return peakOutput;
  }

  /**
   * Sets whether PID control is enabled
   * @param value  true to enable PID control; else false to disable PID control
   */
  public void setEnabled(boolean value) {
    enabled = value;
    m_gainsModified = true;
  }

  /**
   * Sets the proportional gain and marks gains as modified
   * @param value  Gain value to assign
   */
  public void setkP(double value) {
    kP = value;
    m_gainsModified = true;
  }

  /**
   * Sets the integral gain and marks gains as modified
   * @param value  Gain value to assign
   */
  public void setkI(double value) {
    kI = value;
    m_gainsModified = true;
  }

  /**
   * Sets the derivative gain and marks gains as modified
   * @param value  Gain value to assign
   */
  public void setkD(double value) {
    kD = value;
    m_gainsModified = true;
  }

  /**
   * Sets the feed-forward gain and marks gains as modified
   * @param value  Gain value to assign
   */
  public void setkF(double value) {
    kF = value;
    m_gainsModified = true;
  }

  /**
   * Sets the integral zone and marks the Gains object as modified
   * @param value  Gain value to assign
   */
  public void setiZone(int value) {
    iZone = value;
    m_gainsModified = true;
  }

  /**
   * Sets the maximum control output and marks the Gains object as modified
   * @param value  Gain value to assign
   */
  public void setPeakOutput(double value) {
    peakOutput = value;
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
   * Internal method that returns the integral zone value as a double
   */
  private double getiZoneAsDouble() {
    return (double) iZone;
  }

  /** 
   * Internal method that sets the integral zone value from a double
   */
  private void setiZoneAsDouble(double value) {
    iZone = (int) value;
    m_gainsModified = true;
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
    builder.addDoubleProperty("iZone", this::getiZoneAsDouble, this::setiZoneAsDouble);
    builder.addDoubleProperty("maxOut", this::getPeakOutput, this::setPeakOutput);
  }

}
