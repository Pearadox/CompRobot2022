// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax.PearadoxNeo;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private static Drivetrain INSTANCE;

  public final CANSparkMax leftMotor1 =
      new PearadoxNeo(DrivetrainConstants.FRONT_LEFT_MOTOR, IdleMode.kBrake);
  private final CANSparkMax leftMotor2 =
      new PearadoxNeo(DrivetrainConstants.BACK_LEFT_MOTOR, IdleMode.kCoast);
  private final CANSparkMax rightMotor1 =
      new PearadoxNeo(DrivetrainConstants.FRONT_RIGHT_MOTOR, IdleMode.kBrake);
  private final CANSparkMax rightMotor2 =
      new PearadoxNeo(DrivetrainConstants.BACK_RIGHT_MOTOR, IdleMode.kCoast);
  
  private final RelativeEncoder leftEncoder1 = leftMotor1.getEncoder();
  private final RelativeEncoder leftEncoder2 = leftMotor2.getEncoder();
  private final RelativeEncoder rightEncoder1 = rightMotor1.getEncoder();
  private final RelativeEncoder rightEncoder2 = rightMotor2.getEncoder();

  private Drivetrain() {

    // rightMotor2.setInverted(true);
    // leftMotor2.setInverted(true);

    leftMotor2.follow(leftMotor1);
    rightMotor2.follow(rightMotor1);

    if (!SmartDashboard.containsKey("LeftMotor")) SmartDashboard.putNumber("LeftMotor", 0);
    if (!SmartDashboard.containsKey("RightMotor")) SmartDashboard.putNumber("RightMotor", 0);
  }

  public void setVoltages(double left, double right) {
    leftMotor1.setVoltage(left);
    rightMotor1.setVoltage(right);
  }

  public void setSpeed(double left, double right) {
    leftMotor1.set(left);
    rightMotor1.set(right);
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

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds();
  }

  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  public void arcadeDrive(double throttle, double twist) {
    if (throttle < 0.1 && throttle > -0.1) {
      throttle = 0;
    }
    if (twist < 0.15 && twist > -0.15) {
      twist = 0;
    }

    double leftOutput = Math.sin(throttle + twist) * Math.pow(throttle + twist, 2);
    double rightOutput = Math.sin(throttle - twist) * Math.pow(throttle - twist, 2);

    setSpeed(-leftOutput * 0.75, rightOutput * 0.75);
  }

  private final DifferentialDriveOdometry m_odometry =
      new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

  public void zeroHeading() {
    m_gyro.reset();
  }

  public void resetOdometry(Pose2d pose) {
    encoderReset();
    zeroHeading();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(-getHeading()));
  }

  public void encoderReset() {
    leftEncoder1.setPosition(0);
    leftEncoder2.setPosition(0);
    rightEncoder1.setPosition(0);
    rightEncoder2.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(m_gyro.getRotation2d(), getLeftEncoderDistance(), getRightEncoderDistance());
  }

  public void dashboard() {
    SmartDashboard.putNumber("LeftMotorVoltage", leftMotor1.getBusVoltage());
    SmartDashboard.putNumber("RightMotorVoltage", rightMotor1.getBusVoltage());
  }

  public static Drivetrain getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Drivetrain();
    }
    return INSTANCE;
  }
}
