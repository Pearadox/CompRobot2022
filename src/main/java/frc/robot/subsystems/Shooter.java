// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.fasterxml.jackson.databind.ser.std.NumberSerializers.ShortSerializer;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.LimeLight;
import frc.lib.util.LerpTable;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private static Shooter INSTANCE;

  public TalonFX leftShooter = new TalonFX(ShooterConstants.LEFT_SHOOTER);
  public TalonFX rightShooter = new TalonFX(ShooterConstants.RIGHT_SHOOTER);
  private SupplyCurrentLimitConfiguration limitCurrent;
  private double lowGoal = 0.3;
  private double highGoal = 0.65;

  private double prevGoal = highGoal;

  private MedianFilter tyFilter = new MedianFilter(5);
  public NetworkTable llTable = NetworkTableInstance.getDefault().getTable("limelight");
  private LerpTable shooterLerp = new LerpTable();
  private SlewRateLimiter shooterLimiter = new SlewRateLimiter(10);
  private double target = 0;
  private Mode mode = Mode.kAuto;
  private double adjust = 0;
  private double increment = 0.01;
  private double ty;
  
  public enum Mode {
    kAuto, kFixedHigh, kFixedLow;
  }

  /** Creates a new Shooter. */
  public Shooter() {
    leftShooter.setNeutralMode(NeutralMode.Coast);
    rightShooter.setNeutralMode(NeutralMode.Coast);
    leftShooter.setInverted(InvertType.InvertMotorOutput);
    rightShooter.setInverted(InvertType.None);
    if(!SmartDashboard.containsKey("MaxPercentage")) SmartDashboard.putNumber("MaxPercentage", ShooterConstants.MAXPERCENT);
    if(!SmartDashboard.containsKey("SetVoltage")) SmartDashboard.putNumber("SetVoltage", 5.25);
    if(!SmartDashboard.containsKey("Lerp Table")) SmartDashboard.putNumber("Lerp Target", target);
    if(!SmartDashboard.containsKey("Shooter Target RPM")) SmartDashboard.putNumber("Shooter Target RPM", 0);
    limitCurrent = new SupplyCurrentLimitConfiguration(true, 60, 60, 1);
    leftShooter.configSupplyCurrentLimit(limitCurrent);
    rightShooter.configSupplyCurrentLimit(limitCurrent);
    leftShooter.enableVoltageCompensation(true);
    rightShooter.enableVoltageCompensation(true);
    leftShooter.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, Constants.TIMEOUT);
    rightShooter.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, Constants.TIMEOUT);

    // LOOKUP TABLE (LIMELIGHT TY, VOLTAGE)
    shooterLerp.addPoint(-1.32, ShooterConstants.ShooterAdjust * (3.93 - 0*ShooterConstants.kS)/ ShooterConstants.kV);
    shooterLerp.addPoint(-5.25, ShooterConstants.ShooterAdjust * (4.06 - 0*ShooterConstants.kS) / ShooterConstants.kV);    
    // shooterLerp.addPoint(1, 3.6);
    // shooterLerp.addPoint(3.9, ShooterConstants.ShooterAdjust * (3.75 - ShooterConstants.kS) / ShooterConstants.kV);
    shooterLerp.addPoint(-7.92, ShooterConstants.ShooterAdjust * (4.16 - 0*ShooterConstants.kS) / ShooterConstants.kV);
    shooterLerp.addPoint(-10.52, ShooterConstants.ShooterAdjust * (4.32 - 0*ShooterConstants.kS) / ShooterConstants.kV);
    shooterLerp.addPoint(-13.03, ShooterConstants.ShooterAdjust * (4.49 - 0*ShooterConstants.kS) / ShooterConstants.kV);
    shooterLerp.addPoint(-15.45, ShooterConstants.ShooterAdjust * (4.57 - 0*ShooterConstants.kS) / ShooterConstants.kV);
    shooterLerp.addPoint(-17.15, ShooterConstants.ShooterAdjust * (4.63 - 0*ShooterConstants.kS) / ShooterConstants.kV);
    shooterLerp.addPoint(-19.05, ShooterConstants.ShooterAdjust * (4.8 - 0*ShooterConstants.kS) / ShooterConstants.kV);
    shooterLerp.addPoint(-20.62, ShooterConstants.ShooterAdjust * (4.92 - 0*ShooterConstants.kS) / ShooterConstants.kV);
    shooterLerp.addPoint(-22.05, ShooterConstants.ShooterAdjust * (5.0 - 0*ShooterConstants.kS) / ShooterConstants.kV);
    shooterLerp.addPoint(-22.94, ShooterConstants.ShooterAdjust * (5.07 - 0*ShooterConstants.kS) / ShooterConstants.kV);
  }

  public void setMode(Mode mode) {
    this.mode = mode;
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

  public Mode getMode(){
    return mode;
  }

  public Mode getLowMode(){
    return Mode.kFixedLow;
  }

  public void setVoltage(double voltage) {
    var limitedVoltage = shooterLimiter.calculate(voltage);
    leftShooter.set(ControlMode.PercentOutput, limitedVoltage/leftShooter.getBusVoltage());
    rightShooter.set(ControlMode.PercentOutput, limitedVoltage/rightShooter.getBusVoltage());
  }

  public void setSetpoint(double rps) {
    setVoltage(ShooterConstants.kS + ShooterConstants.kV * rps + ShooterConstants.kP * (rps - getSpeed()));
  }

  public void autoSpeed() {
    setSetpoint(target);
  }

  //Negative is Out, Positive is In
  public void setSpeed(double speed) {
    leftShooter.set(ControlMode.PercentOutput, -speed);
    rightShooter.set(ControlMode.PercentOutput, -speed);
  }

  public double getPercentOutput() {
    return (leftShooter.getMotorOutputPercent() + rightShooter.getMotorOutputPercent())/2;
  }

  public void setLeds(int state) {
    llTable.getEntry("ledMode").setNumber(state);
  }

  public double getSpeed() {
    return (leftShooter.getSelectedSensorVelocity() + rightShooter.getSelectedSensorVelocity()) / 409.6;
  }


  public void dashboard() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter PercentOutput", getPercentOutput());
    SmartDashboard.putNumber("Shooter Current", leftShooter.getSupplyCurrent());
    SmartDashboard.putNumber("Shooter Temp", leftShooter.getTemperature());
    SmartDashboard.putNumber("Shooter Bus Voltage", leftShooter.getBusVoltage());
    SmartDashboard.putString("Shooter Mode", mode.toString());
    SmartDashboard.putNumber("Shooter RPS", getSpeed()
    );


    // if(leftShooter.getTemperature() > 50) {
    // limitCurrent = new SupplyCurrentLimitConfiguration(true, 15, 15, 2);
    // leftShooter.configSupplyCurrentLimit(limitCurrent);
    // rightShooter.configSupplyCurrentLimit(limitCurrent);
    // }

    // if (llTable.getEntry("ta").getDouble(0) > 0 && mode == Mode.kAuto) {
    //   ty = tyFilter.calculate(llTable.getEntry("ty").getDouble(0));
    //   target = shooterLerp.interpolate(ty);
    // } else if (mode == Mode.kFixedLow) {
    //   target = 3.5 / ShooterConstants.kV;
    // } else {
    //   target = SmartDashboard.getNumber("Shooter Target RPM", 0);
    // }

    if (llTable.getEntry("ta").getDouble(0) > 0 && mode == Mode.kAuto) {
      ty = tyFilter.calculate(llTable.getEntry("ty").getDouble(0));
      target = shooterLerp.interpolate(ty);
    } else if (mode == Mode.kFixedLow) {
      target = 3.5 / ShooterConstants.kV;
    } else {
      target = SmartDashboard.getNumber("SetVoltage", 5.25)/ ShooterConstants.kV;
    }
    // SmartDashboard.putNumber("Target", target);
    SmartDashboard.putNumber("Lerp Target", target);
  }

  public double getTarget() {
    return target;
  }

  public static Shooter getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Shooter();
    }
    return INSTANCE;
  }

}
