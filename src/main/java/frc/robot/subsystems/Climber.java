// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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
  private DoubleSolenoid leftSolenoid;
  private DoubleSolenoid rightSolenoid;
  // private Encoder liftEncoder;

  public Object climbUp;


  /** Creates a new Climber. */
  public Climber() {
    //assigns motors to the falcon controller   
    leftLiftMotor = new TalonFX(ClimberConstants.LEFT_LIFT_MOTOR);
    rightLiftMotor = new TalonFX(ClimberConstants.RIGHT_LIFT_MOTOR);
    leftLiftMotor.setNeutralMode(NeutralMode.Brake);
    rightLiftMotor.setNeutralMode(NeutralMode.Brake);
    // liftMotor = new Encoder
    // leftSolenoid = new Solenoid(PneumaticsModuleType.REVPH, ClimberConstants.LEFT_SOLENOID);
    leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimberConstants.LEFT_FOR_SOLENOID, ClimberConstants.LEFT_REV_SOLENOID);
    rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimberConstants.RIGHT_FOR_SOLENOID, ClimberConstants.RIGHT_REV_SOLENOID);
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

  public void climbOpenSol() {
    leftSolenoid.set(Value.kForward);
    rightSolenoid.set(Value.kForward);
  }

  public void climbCloseSol() {
    leftSolenoid.set(Value.kReverse);
    rightSolenoid.set(Value.kReverse);
  }
 
  public void climbToggleSol(){
    leftSolenoid.toggle();;
    rightSolenoid.toggle();
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
