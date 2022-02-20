// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private static Intake INSTANCE;

  private final TalonFX intakeMotor;
  private final DoubleSolenoid intakeSolenoid;


  public Intake() {
    intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR);
    intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, IntakeConstants.INTAKE_FOR_SOLENOID, IntakeConstants.INTAKE_REV_SOLENOID);
  }


  public void setSpeed(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setIntakeIn() {
    setSpeed(0.5);
  }

  public void setIntakeOut() {
    setSpeed(-0.5);
  }

  public void intakeOpenSol() {
    intakeSolenoid.set(Value.kForward);
  }

  public void intakeCloseSol() {
    intakeSolenoid.set(Value.kReverse);
  }

  public void intakeToggleSol() {
    intakeSolenoid.toggle();
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
