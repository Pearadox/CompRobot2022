// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransportConstants;

public class Transport extends SubsystemBase {
  private static Transport INSTANCE;

  private final CANSparkMax topTransportMotor;
  private final CANSparkMax botTransportMotor;

  /** Creates a new Transport. */
  public Transport() {
    topTransportMotor = new CANSparkMax(TransportConstants.TOP_TRANSPORT_MOTOR, MotorType.kBrushless);
    botTransportMotor = new CANSparkMax(TransportConstants.BOT_TRANSPORT_MOTOR, MotorType.kBrushless);
  }

  public void setSpeed(double speed) {
    topTransportMotor.set(speed);
    botTransportMotor.set(speed);
  }

  public void transportIn() {
    setSpeed(0.5);
  }

  public void transportOut() {
    setSpeed(-0.5);
  }

  public void dashboard() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static Transport getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Transport();
    }
    return INSTANCE;
  }

}
