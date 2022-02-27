// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private static Shooter INSTANCE;

  public TalonFX leftShooter = new TalonFX(ShooterConstants.LEFT_SHOOTER);
  public TalonFX rightShooter = new TalonFX(ShooterConstants.RIGHT_SHOOTER);
  private SupplyCurrentLimitConfiguration limitCurrent;
  private double lowGoal = 0.3;
  private double highGoal = 0.65;
  private double prevGoal = highGoal;


  /** Creates a new Shooter. */
  public Shooter() {
    leftShooter.setNeutralMode(NeutralMode.Coast);
    rightShooter.setNeutralMode(NeutralMode.Coast);
    leftShooter.setInverted(InvertType.InvertMotorOutput);
    if(!SmartDashboard.containsKey("MaxPercentage")) SmartDashboard.putNumber("MaxPercentage", ShooterConstants.MAXPERCENT);
    limitCurrent = new SupplyCurrentLimitConfiguration(true, 80, 60, 1);
    leftShooter.configSupplyCurrentLimit(limitCurrent);
    rightShooter.configSupplyCurrentLimit(limitCurrent);
  }

  public double getPreviousGoal() {
    return prevGoal;
  }

  public void setPreviousGoal() {
    prevGoal = SmartDashboard.getNumber("MaxPercentage", prevGoal);
  }

  public double getLow() {
    return lowGoal;
  }

  public double getHigh() {
    return highGoal;
  }


  //Negative is Out, Positive is In
  public void setSpeed(double speed) {
    leftShooter.set(ControlMode.PercentOutput, -speed);
    rightShooter.set(ControlMode.PercentOutput, -speed);
  }

  public double getPercentOutput() {
    return (leftShooter.getMotorOutputPercent() + rightShooter.getMotorOutputPercent())/2;
  }

  public void dashboard() {
    SmartDashboard.putNumber("Shooter PercentOutput", getPercentOutput());
    SmartDashboard.putNumber("Shooter Current", leftShooter.getSupplyCurrent());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter PercentOutput", getPercentOutput());
    SmartDashboard.putNumber("Shooter Current", leftShooter.getSupplyCurrent());
    SmartDashboard.putNumber("Shooter Temp", leftShooter.getTemperature());
    // if(leftShooter.getTemperature() > 50) {
    // limitCurrent = new SupplyCurrentLimitConfiguration(true, 15, 15, 2);
    // leftShooter.configSupplyCurrentLimit(limitCurrent);
    // rightShooter.configSupplyCurrentLimit(limitCurrent);
    // }
  }

  public static Shooter getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Shooter();
    }
    return INSTANCE;
  }

}
