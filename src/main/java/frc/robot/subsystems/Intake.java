// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private static Intake INSTANCE;

  private final TalonFX intakeTopMotor;
  private final TalonFX intakeBottomMotor;
  private final DoubleSolenoid intakeSolenoid;
  private SlewRateLimiter limiter; 


  public Intake() {
    intakeTopMotor = new TalonFX(IntakeConstants.INTAKE_RIGHT_MOTOR);
    intakeBottomMotor = new TalonFX(IntakeConstants.INTAKE_LEFT_MOTOR);
    intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, IntakeConstants.INTAKE_FOR_SOLENOID, IntakeConstants.INTAKE_REV_SOLENOID);
    SmartDashboard.putString("Intake Value", getIntakeValue() + "");
    intakeTopMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, 1));
    intakeTopMotor.setNeutralMode(NeutralMode.Coast);
    intakeTopMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, Constants.TIMEOUT);
    intakeBottomMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, 1));
    intakeBottomMotor.setNeutralMode(NeutralMode.Coast);
    intakeBottomMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, Constants.TIMEOUT);
    limiter = new SlewRateLimiter(0.25, 0);
  }

  //Bottom In is Positive
  //Top In is negative

  public void setSpeed(double botspeed, double topspeed) {
    intakeBottomMotor.set(ControlMode.PercentOutput, botspeed);
    intakeTopMotor.set(ControlMode.PercentOutput, -topspeed);
  }

  public void setIntakeIn(double speed) {
    setSpeed(speed, speed);
  }

  public double getIntakeCurrent(){
    return intakeTopMotor.getSupplyCurrent();
  }

  public void setIntakeOut() {
    if(intakeSolenoid.get() != Value.kReverse){
      setSpeed(0.5, -0.5);
    }
    else{
      setSpeed(0, -0.5);
    }
  }

  public void intakeOpenSol() {
    intakeSolenoid.set(Value.kForward);
  }

  public void intakeCloseSol() {
    intakeSolenoid.set(Value.kReverse);
  }

  public void intakeShoot() {
    if(getIntakeValue() == DoubleSolenoid.Value.kOff || getIntakeValue() == DoubleSolenoid.Value.kForward) {
      intakeCloseSol();
    }
  }
  
  public void intakeToggleSol() {
    if(getIntakeValue() == DoubleSolenoid.Value.kOff || getIntakeValue() == DoubleSolenoid.Value.kReverse) {
      intakeOpenSol();
    }
    else {
      intakeCloseSol();
    }
  }

  public Value getIntakeValue() {
    return intakeSolenoid.get();
  }

  public void stop() {
    setSpeed(0,0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Intake Value", getIntakeValue() + "");
    SmartDashboard.putNumber("intake top current", intakeTopMotor.getSupplyCurrent());
    SmartDashboard.putNumber("intake bot current", intakeBottomMotor.getSupplyCurrent());
    SmartDashboard.putNumber("Intake Temp", intakeTopMotor.getTemperature());
  }

  public void dashboard() {}

  public static Intake getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Intake();
    }
    return INSTANCE;
  }
}
