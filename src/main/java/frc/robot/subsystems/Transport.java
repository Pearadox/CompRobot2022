// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.TransportConstants;

public class Transport extends SubsystemBase {
  private static Transport INSTANCE;

  private final CANSparkMax topTransportMotor;
  private final CANSparkMax botTransportMotor;
  public TalonFX feeder;
  // private final TalonFX beterTransportMotor;

  public boolean hasBall = false;

  /** Creates a new Transport. */
  public Transport() {
    topTransportMotor = new CANSparkMax(TransportConstants.TOP_TRANSPORT_MOTOR, MotorType.kBrushless);
    botTransportMotor = new CANSparkMax(TransportConstants.BOT_TRANSPORT_MOTOR, MotorType.kBrushless);
    feeder = new TalonFX(TransportConstants.BETER_TRANSPORT_MOTOR);
    feeder.setNeutralMode(NeutralMode.Brake);
    topTransportMotor.restoreFactoryDefaults();
    botTransportMotor.restoreFactoryDefaults();
    topTransportMotor.setIdleMode(IdleMode.kBrake);
    botTransportMotor.setIdleMode(IdleMode.kBrake);
    topTransportMotor.setSmartCurrentLimit(20);
    botTransportMotor.setSmartCurrentLimit(20);
    topTransportMotor.burnFlash();
    botTransportMotor.burnFlash();
  }

  public void setSpeed(double speed) {
    topTransportMotor.set(-speed/2);
    botTransportMotor.set(-speed);
  }

  public void setBotSpeed(double speed){
    botTransportMotor.set(-speed);
  }

  public void stopTopMotor(){
    topTransportMotor.set(0);
  }

  public void transportIn() {
    if (!hasBall) {
      topTransportMotor.set(-0.4);
    } else {
      topTransportMotor.set(0);
    }
    botTransportMotor.set(-0.5);
  }

  public void transportOut() {
    botTransportMotor.set(1.0);
    topTransportMotor.set(0.65);
  }

  public void transportStop() {
    setSpeed(0);
    topTransportMotor.set(0);
    feeder.set(ControlMode.PercentOutput, 0);
  }

  public double getTopCurrent(){
    return topTransportMotor.getOutputCurrent();
  }

  public void feederShoot(){
    topTransportMotor.set(-0.65);
    botTransportMotor.set(-0.9);
    feeder.set(ControlMode.PercentOutput, 1);
  }

  public void clearBall() {
    hasBall = false;
  }

  public void detectBall() {
    hasBall = true;
  }

  public void feederHold() {
    if (!hasBall) {
      feeder.set(ControlMode.PercentOutput, -0.8);

    } else {
      feeder.set(ControlMode.PercentOutput, 0);
    }
  }

  public void dashboard() {
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("top Transport Current", topTransportMotor.getOutputCurrent());
    SmartDashboard.putNumber("bot Transport Current", botTransportMotor.getOutputCurrent());
    SmartDashboard.putNumber("feeder Transport Current", feeder.getSupplyCurrent());
    SmartDashboard.putBoolean("Has Ball", hasBall);

    if(RobotContainer.colorSensor.getRawColor0().red - RobotContainer.colorSensor.getRawColor0().blue > 2000){
      SmartDashboard.putString("Color", "Red");
    }
    else if(RobotContainer.colorSensor.getRawColor0().blue - RobotContainer.colorSensor.getRawColor0().red > 2000){
      SmartDashboard.putString("Color", "Blue");
    }
    else{
      SmartDashboard.putString("Color", "None");
    }
    
    // if (topTransportMotor.get() != 0 && feeder.getSupplyCurrent() > 4) {
    //   hasBall = true;
    // }
  }

  public static Transport getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Transport();
    }
    return INSTANCE;
  }

}
