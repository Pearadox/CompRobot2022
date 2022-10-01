// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimberConstants;
import frc.robot.commands.ClimberZero;
import frc.robot.commands.CompressClimberSol;
import frc.robot.commands.ExpandClimberSol;
import frc.robot.commands.SetClimb;
import frc.robot.commands.SetExtend;
import frc.robot.commands.SetFirstExtend;
import frc.robot.commands.SetMidRung;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

public class Climber extends SubsystemBase {
  private static Climber INSTANCE;

  public final TalonFX leftLiftMotor;
  public final TalonFX rightLiftMotor;
  private DoubleSolenoid leftSolenoid;
  private DoubleSolenoid rightSolenoid;

  private int count = 0;
  private int rescount = -1;
  private boolean stopping = false;

  /** Creates a new Climber. */
  public Climber() {
    //assigns motors to the falcon controller   
    leftLiftMotor = new TalonFX(ClimberConstants.LEFT_LIFT_MOTOR);
    rightLiftMotor = new TalonFX(ClimberConstants.RIGHT_LIFT_MOTOR);
    
    leftLiftMotor.configFactoryDefault();
    rightLiftMotor.configFactoryDefault();

    leftLiftMotor.setNeutralMode(NeutralMode.Brake);
    rightLiftMotor.setNeutralMode(NeutralMode.Brake);

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
  
    leftLiftMotor.configSupplyCurrentLimit(
      new SupplyCurrentLimitConfiguration(true, 120, 150, 0.1)
    );
    rightLiftMotor.configSupplyCurrentLimit(
      new SupplyCurrentLimitConfiguration(true, 120, 150, 0.1)
    );


    if(!SmartDashboard.containsKey("Left Lift Encoder")) SmartDashboard.putNumber("Left Lift Encoder", getLeftLiftEncoder()); //77.58
    if(!SmartDashboard.containsKey("Right Lift Encoder")) SmartDashboard.putNumber("Right Lift Encoder", getRightLiftEncoder()); //-80.84
    if(!SmartDashboard.containsKey("Left Position")) SmartDashboard.putNumber("Left Position", leftLiftMotor.getSelectedSensorPosition()); //77.58
    if(!SmartDashboard.containsKey("Right Position")) SmartDashboard.putNumber("Right Position", rightLiftMotor.getSelectedSensorPosition()); //-80.84

    SmartDashboard.putData("Climber", this);

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
    setClimbMotor(-0.8);
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
    leftLiftMotor.set(TalonFXControlMode.Position, pos, DemandType.ArbitraryFeedForward, 0.05);
  }
  public void setRightPosition(double pos) {
    rightLiftMotor.set(TalonFXControlMode.Position, pos, DemandType.ArbitraryFeedForward, -0.05);
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
    SmartDashboard.putNumber("Left Current", leftLiftMotor.getSupplyCurrent());
    SmartDashboard.putNumber("Right Current", rightLiftMotor.getSupplyCurrent());
    SmartDashboard.putData("Climber", this);
    SmartDashboard.putNumber("Climber Sequence", count);
    SmartDashboard.putNumber("Left Error", getLeftError());
    SmartDashboard.putNumber("Right Error", getRightError());

    if(RobotContainer.getOperJoystick().getPOV() != -1 && (RobotContainer.getOperJoystick().getPOV() > 314 || RobotContainer.getOperJoystick().getPOV() < 46)) {
      restoreSequence();
    } else if (RobotContainer.getOperJoystick().getPOV() != -1 && (RobotContainer.getOperJoystick().getPOV() > 134 && RobotContainer.getOperJoystick().getPOV() < 226)) {
      stopSequence();
    }
  }

  public static Climber getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Climber();
    }
    return INSTANCE;
  }

  public void incrementSequence() {
    switch (count) {
      case 0: 
        CommandScheduler.getInstance().schedule(new ClimberZero());
        // count++;
        break;
      case 1:
        CommandScheduler.getInstance().schedule(new SetFirstExtend());
        break;
      case 2:
        CommandScheduler.getInstance().schedule(new SetClimb());
        break;
      case 3:
        CommandScheduler.getInstance().schedule(new RunCommand(() -> this.setClimbMotor(0.2)).withTimeout(0.5).andThen(new SetMidRung().withTimeout(0.25)).andThen(new CompressClimberSol()));
        break;
      case 4:
        CommandScheduler.getInstance().schedule(new SetExtend());
        break;
      case 5:
        CommandScheduler.getInstance().schedule(new ExpandClimberSol());
        break;
      case 6:
        CommandScheduler.getInstance().schedule(new SetClimb());
        break;
      case 7:
      CommandScheduler.getInstance().schedule(new RunCommand(() -> this.setClimbMotor(0.2)).withTimeout(0.5).andThen(new SetMidRung().withTimeout(0.25)).andThen(new CompressClimberSol()));
        break;
      case 8:
      CommandScheduler.getInstance().schedule(new SetExtend());
        break;
      case 9:
      CommandScheduler.getInstance().schedule(new ExpandClimberSol());
      break;
      case 10:
        CommandScheduler.getInstance().schedule(new SetClimb().withTimeout(1.0));
        break;
      case 11:
        CommandScheduler.getInstance().schedule(new RunCommand(() ->  this.setClimbMotor(0)).withTimeout(0.04)
        .andThen(new InstantCommand(() -> this.flashlightOn())));
    }
    count++;
  }

  public void restoreSequence() {
    if (rescount != -1) {
      stopping = false;
      count = rescount;
    }
  }

  public void stopSequence() {
    if(count < 13) {
      rescount = count;
    }
    stopping = true;
    count = 13;
    incrementSequence();
  }

  public boolean getStopping() {
    return stopping;
  }

  public void resetSequence() {
    count = 0;
  }

  public void flashlightOn(){
    RobotContainer.pdh.setSwitchableChannel(true);
  }

  public void flashlightOff(){
    RobotContainer.pdh.setSwitchableChannel(false);
  }
}
