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

/**
 * A class that wraps PID gains and parameters for a CTRE Falcon motor
 */
public class PIDGains extends Object {
  /** true if PID control is enabled; else false if it is disabled */
  public boolean m_enabled;
  /** Proportional gain */
  public double m_kP;
  /** Integral gain */
  public double m_kI;
  /** Derivative gain */
  public double m_kD;
  /** Feed-forward gain */
  public double m_kF;

  /** Creates uninitialized default Gains */
  public PIDGains() {
    m_enabled = false;
    m_kP = m_kI = m_kD = m_kF = 0.0;
  }

  /**
   * Creates an instance of the object with given parameter values
   * 
   * @param enabled True if the controller is enabled; else false to disable it
   * @param kP      Proportional gain
   * @param kI      Integral gain
   * @param kD      Derivative gain
   * @param kF      Feed-forward gain
   */
  public PIDGains(boolean enabled, double kP, double kI, double kD, double kF) {
    m_enabled = enabled;
    m_kP = kP;
    m_kI = kI;
    m_kD = kD;
    m_kF = kF;
  }

  /** 
   * Copy values from another PIDGains object
   * @param other  Gains to copy
   */
  public void copy(PIDGains other) {
    m_enabled = other.m_enabled;
    m_kP = other.m_kP;
    m_kI = other.m_kI;
    m_kD = other.m_kD;
    m_kF = other.m_kF;
  }

  /**
   * Creates an instance of the object using values copied from another object
   * 
   * @param other Gains object to copy values from
   */
  public PIDGains(PIDGains other) {
    m_enabled = other.m_enabled;
    m_kP = other.m_kP;
    m_kI = other.m_kI;
    m_kD = other.m_kD;
    m_kF = other.m_kF;
  }

  /**
   * Enables/disables PID control
   * 
   * @param enabled true if PID control is enabled; else false for disabled
   */
  public void enabled(boolean enabled) {
    m_enabled = enabled;
  }

  /** @return true if PID control is enabled; else false */
  public boolean enabled() {
    return m_enabled;
  }

  /**
   * Sets proportional gain (kP)
   * 
   * @param gain Gain value to apply
   */
  public void kP(double gain) {
    m_kP = gain;
  }

  /** @return the proportional gain (kP) */
  public double kP() {
    return m_kP;
  }

  /**
   * Sets integral gain (kI)
   * 
   * @param gain Gain value to apply
   */
  public void kI(double gain) {
    m_kI = gain;
  }

  /** @return the integral gain (kI) */
  public double kI() {
    return m_kI;
  }

  /**
   * Sets derivative gain (kD)
   * 
   * @param gain Gain value to apply
   */
  public void kD(double gain) {
    m_kD = gain;
  }

  /** @return the derivative gain (kD) */
  public double kD() {
    return m_kD;
  }

  /**
   * Sets feed-forward gain (kF)
   * 
   * @param gain Gain value to apply
   */
  public void kF(double gain) {
    m_kF = gain;
  }

  /** @return the feed-forward gain (kF) */
  public double kF() {
    return m_kF;
  }
}
