// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

public class Climber extends SubsystemBase {
  private static Climber INSTANCE;

  private final TalonFX leftLiftMotor;
  private final TalonFX rightLiftMotor;
  private DoubleSolenoid leftSolenoid;
  private DoubleSolenoid rightSolenoid;

  /** Creates a new Climber. */
  public Climber() {
    //assigns motors to the falcon controller   
    leftLiftMotor = new TalonFX(ClimberConstants.LEFT_LIFT_MOTOR);
    rightLiftMotor = new TalonFX(ClimberConstants.RIGHT_LIFT_MOTOR);
    
    leftLiftMotor.configFactoryDefault();
    rightLiftMotor.configFactoryDefault();

    leftLiftMotor.setNeutralMode(NeutralMode.Brake);
    leftLiftMotor.setSensorPhase(Constants.ClimberConstants.kLeftSensorPhase);
    leftLiftMotor.setInverted(Constants.ClimberConstants.kLeftMotorInvert);
    leftLiftMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
    leftLiftMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
    leftLiftMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
    leftLiftMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);
    leftLiftMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    leftLiftMotor.config_kP(Constants.ClimberConstants.kPIDLoopIdx,  Constants.ClimberConstants.kP, Constants.kTimeoutMs);

    rightLiftMotor.setNeutralMode(NeutralMode.Brake);
    rightLiftMotor.setSensorPhase(Constants.ClimberConstants.kRightSensorPhase);
    rightLiftMotor.setInverted(Constants.ClimberConstants.kRightMotorInvert);
    rightLiftMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
    rightLiftMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
    rightLiftMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
    rightLiftMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);
    rightLiftMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    rightLiftMotor.config_kP(Constants.ClimberConstants.kPIDLoopIdx,  Constants.ClimberConstants.kP, Constants.kTimeoutMs);
    rightLiftMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
  


    if(!SmartDashboard.containsKey("Left Lift Encoder")) SmartDashboard.putNumber("Left Lift Encoder", getLeftLiftEncoder()); //77.58
    if(!SmartDashboard.containsKey("Right Lift Encoder")) SmartDashboard.putNumber("Right Lift Encoder", getRightLiftEncoder()); //-80.84
    if(!SmartDashboard.containsKey("Left Position")) SmartDashboard.putNumber("Left Position", leftLiftMotor.getSelectedSensorPosition()); //77.58
    if(!SmartDashboard.containsKey("Right Position")) SmartDashboard.putNumber("Right Position", rightLiftMotor.getSelectedSensorPosition()); //-80.84

    leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimberConstants.LEFT_FOR_SOLENOID, ClimberConstants.LEFT_REV_SOLENOID);
    rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimberConstants.RIGHT_FOR_SOLENOID, ClimberConstants.RIGHT_REV_SOLENOID);
  }

  public void setLeftLiftMotor(double percent) {
    leftLiftMotor.set(ControlMode.PercentOutput, -percent);
  }

  public void setRightLiftMotor(double percent) {
    rightLiftMotor.set(ControlMode.PercentOutput, percent);
  }

  public void setClimbMotor(double percent) {
    setLeftLiftMotor(percent);
    setRightLiftMotor(percent);
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
    leftSolenoid.toggle();
    rightSolenoid.toggle();
  }

  public double getLeftLiftEncoder() {
    return -leftLiftMotor.getSelectedSensorPosition() / 2048;
  }

  public double getRightLiftEncoder() {
    return rightLiftMotor.getSelectedSensorPosition() / 2048;
  }

  public void reset() {
    leftLiftMotor.setSelectedSensorPosition(0);
    rightLiftMotor.setSelectedSensorPosition(0);
  }

  public void dashboard() {
  }
  
  public void setLeftPosition(double pos) {
    leftLiftMotor.set(TalonFXControlMode.Position, pos);
  }
  public void setRightPosition(double pos) {
    rightLiftMotor.set(TalonFXControlMode.Position, pos);
  }
  
  public double getLeftError() {
    return leftLiftMotor.getClosedLoopError();
  }
  public double getRightError() {
    return rightLiftMotor.getClosedLoopError();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Lift Encoder", getLeftLiftEncoder());
    SmartDashboard.putNumber("Right Lift Encoder", getRightLiftEncoder());
    SmartDashboard.putNumber("Left Position", leftLiftMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Position", rightLiftMotor.getSelectedSensorPosition());
  }

  public static Climber getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Climber();
    }
    return INSTANCE;
  }
}
