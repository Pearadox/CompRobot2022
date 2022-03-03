// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransportConstants;

public class Transport extends SubsystemBase {
  private static Transport INSTANCE;

  private final CANSparkMax topTransportMotor;
  private final CANSparkMax botTransportMotor;
  public TalonFX feeder;
  // private final TalonFX beterTransportMotor;

  private boolean loading = true;

  /** Creates a new Transport. */
  public Transport() {
    topTransportMotor = new CANSparkMax(TransportConstants.TOP_TRANSPORT_MOTOR, MotorType.kBrushless);
    botTransportMotor = new CANSparkMax(TransportConstants.BOT_TRANSPORT_MOTOR, MotorType.kBrushless);
    feeder = new TalonFX(TransportConstants.BETER_TRANSPORT_MOTOR);
  }

  public void setSpeed(double speed) {
    topTransportMotor.set(-speed);
    botTransportMotor.set(-speed);
  }

  public void transportIn() {
  if (loading){
    setSpeed(0.3);
    }
  }

  public void transportOut() {
    setSpeed(-0.5);
  }

  public void transportStop() {
    setSpeed(0);
  }

  public void feederShoot(){
    setSpeed(0.35);
    feeder.set(ControlMode.PercentOutput, 0.35);
  }

  public void toggleIntake(){
    if(loading){
      loading = false;
    }
    else if (!loading){
      loading = true;
    }
  }

  public void feederHold(){
    feeder.set(ControlMode.PercentOutput, -0.6);
  }

  public void dashboard() {
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("top Transport Current", topTransportMotor.getOutputCurrent());
    SmartDashboard.putNumber("bot Transport Current", botTransportMotor.getOutputCurrent());
    SmartDashboard.putNumber("feeder Transport Current", feeder.getSupplyCurrent());
    SmartDashboard.putBoolean("Loading", loading);

  }

  public static Transport getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Transport();
    }
    return INSTANCE;
  }

}
