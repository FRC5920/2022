// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;


/** Add your docs here. */
public class SendableGains extends Gains implements Sendable {

  /**
   * Constructs an object using initial values given in a Gains object
   * 
   * @param val Initial values to assign to the object
   */
  public SendableGains() {
    super();
  }

  /**
   * Constructs an object using initial values given in a Gains object
   * 
   * @param val Initial values to assign to the object
   */
  public SendableGains(Gains val) {
    enabled = val.enabled;
    kP = val.kP;
    kI = val.kI;
    kD = val.kD;
    kF = val.kF;
    iZone = val.iZone;
    peakOutput = val.peakOutput;
  }

  public boolean getEnabled() {
    return enabled;
  }

  public double getkP() {
    return kP;
  }

  public double getkI() {
    return kI;
  }

  public double getkD() {
    return kD;
  }

  public double getkF() {
    return kF;
  }

  public int getiZone() {
    return iZone;
  }

  public double getPeakOutput() {
    return peakOutput;
  }

  public void setEnabled(boolean value) {
    enabled = value;
  }

  public void setkP(double value) {
    kP = value;
  }

  public void setkI(double value) {
    kI = value;
  }

  public void setkD(double value) {
    kD = value;
  }

  public void setkF(double value) {
    kF = value;
  }

  public void setiZone(int value) {
    iZone = value;
  }

  public void setPeakOutput(double value) {
    peakOutput = value;
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

  private double getiZoneAsDouble() {
    return (double) iZone;
  }

  private void setiZoneAsDouble(double value) {
    iZone = (int) value;
  }
}
