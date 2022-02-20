// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class Climber extends SubsystemBase {
  private static Climber INSTANCE;

  private final TalonFX leftLiftMotor;
  private final TalonFX rightLiftMotor;
  private Solenoid leftSolenoid;
  private Solenoid rightSolenoid;
  // private Encoder liftEncoder;

  public Object climbUp;


  /** Creates a new Climber. */
  public Climber() {
    //dania did this section
    //comment 
    leftLiftMotor = new TalonFX(ClimberConstants.LEFT_LIFT_MOTOR);
    rightLiftMotor = new TalonFX(ClimberConstants.RIGHT_LIFT_MOTOR);
    leftLiftMotor.setNeutralMode(NeutralMode.Brake);
    rightLiftMotor.setNeutralMode(NeutralMode.Brake);
    // liftMotor = new Encoder
    leftSolenoid = new Solenoid(PneumaticsModuleType.REVPH, ClimberConstants.LEFT_SOLENOID);
    rightSolenoid = new Solenoid(PneumaticsModuleType.REVPH, ClimberConstants.RIGHT_SOLENOID);
  }

  public void setClimbMotor(double percent) {
    leftLiftMotor.set(ControlMode.PercentOutput, percent);
    rightLiftMotor.set(ControlMode.PercentOutput, -percent);
  }

  public void climbUp() {
    setClimbMotor(0.5);
  }

  public void climbDown() {
    setClimbMotor(-0.5);
  }

  public void stopClimb() {
    setClimbMotor(0);
  }

  public void openSolenoid() {
    leftSolenoid.set(true);
    rightSolenoid.set(true);
  }

  public void closeSolenoid() {
    leftSolenoid.set(false);
    rightSolenoid.set(false);
  }

  public void dashboard() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static Climber getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Climber();
    }
    return INSTANCE;
  }
}
