// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private static Intake INSTANCE;

  private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR, MotorType.kBrushless);

  public Intake() {}

  public void setSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void setIntakeIn() {
    setSpeed(0.3);
  }

  public void setIntakeOut() {
    setSpeed(-0.5);
  }

  public void zero() {
    setSpeed(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void dashboard() {}

  public static Intake getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Intake();
    }
    return INSTANCE;
  }
}
