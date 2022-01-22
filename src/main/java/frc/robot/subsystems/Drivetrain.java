// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.kauailabs.navx.frc.AHRS;
import static frc.robot.Constants.DrivetrainConstants.*;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private static Drivetrain INSTANCE;

  private final CANSparkMax leftMotor1 = new CANSparkMax(11, MotorType.kBrushless);
  private final CANSparkMax leftMotor2 = new CANSparkMax(10, MotorType.kBrushless);
  private final CANSparkMax rightMotor1 = new CANSparkMax(13, MotorType.kBrushless);
  private final CANSparkMax rightMotor2 = new CANSparkMax(12, MotorType.kBrushless);

  private final RelativeEncoder leftEncoder1 = leftMotor1.getEncoder();
  private final RelativeEncoder leftEncoder2 = leftMotor2.getEncoder();
  private final RelativeEncoder rightEncoder1 = rightMotor1.getEncoder();
  private final RelativeEncoder rightEncoder2 = rightMotor2.getEncoder();

  

  private Drivetrain() {
    
    // rightMotor2.setInverted(true);
    // leftMotor2.setInverted(true);

    leftMotor2.follow(leftMotor1);
    rightMotor2.follow(rightMotor1);
  }

  public void setVoltages(double left, double right) {
    leftMotor1.setVoltage(left);
    rightMotor1.setVoltage(right);
  }

  public void setSpeed(double left, double right) {
    leftMotor1.set(left);
    // leftMotor2.set(left);
    rightMotor1.set(right);
    // rightMotor2.set(right);
  }

  public double getLeftEncoderRevs() {
    return (leftEncoder1.getPosition() + leftEncoder2.getPosition());
  }

  public double getLeftEncoderDistance() {
    return getLeftEncoderRevs() * WHEEL_CIRCUMFRENCE / GEAR_RATIO;
  }

  public double getRightEncoderRevs() {
    return (rightEncoder1.getPosition() + rightEncoder2.getPosition());
  }

  public double getRightEncoderDistance() {
    return getRightEncoderRevs() * WHEEL_CIRCUMFRENCE / GEAR_RATIO;
  }
  
  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds();
  }
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  public void arcadeDrive(double throttle, double twist) {
    if(throttle < 0.1 && throttle > -0.1) {
      throttle = 0;
    } 
    if (twist < 0.1 && twist > -0.1) {
      twist = 0;
    }


      double leftOutput = throttle - twist;
      double rightOutput = throttle + twist;
      SmartDashboard.putNumber("LeftMotor", leftOutput);
      SmartDashboard.putNumber("RightMotor", rightOutput);
      setSpeed(leftOutput * 0.25, rightOutput * 0.25);
  }

  // private final DifferentialDriveOdometry m_odometry;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // m_odometry.update(
    //   m_gyro.getRotation2d(), getLeftEncoderDistance(), getRightEncoderDistance());
  }

  public static Drivetrain getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Drivetrain();
    }
    return INSTANCE;
  }
}
