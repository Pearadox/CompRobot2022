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
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax.PearadoxNeo;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private static Drivetrain INSTANCE;

  private Pose2d pose;

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
  public double Vision_kp = 0.0;
  public double Vision_ki = 0.0;
  public double Vision_kd = 0.0;
  public double Vision_dz = 0.3;

  public void setMode(boolean mode){
    if(mode){
      leftMotor1.setIdleMode(IdleMode.kBrake);
      leftMotor2.setIdleMode(IdleMode.kCoast);
      rightMotor1.setIdleMode(IdleMode.kBrake);
      rightMotor2.setIdleMode(IdleMode.kCoast);
    }
    else{
      leftMotor1.setIdleMode(IdleMode.kCoast);
      leftMotor2.setIdleMode(IdleMode.kCoast);
      rightMotor1.setIdleMode(IdleMode.kCoast);
      rightMotor2.setIdleMode(IdleMode.kCoast);
    }
  }

  private Drivetrain() {

    // rightMotor2.setInverted(true);
    leftMotor1.setInverted(true);
    rightMotor1.setInverted(false);

    leftMotor2.follow(leftMotor1);
    rightMotor2.follow(rightMotor1);

    if (!SmartDashboard.containsKey("LeftMotor")) SmartDashboard.putNumber("LeftMotor", 0);
    if (!SmartDashboard.containsKey("RightMotor")) SmartDashboard.putNumber("RightMotor", 0);
    if(!SmartDashboard.containsKey("VisionHold_Kp")) SmartDashboard.putNumber("VisionHold_Kp", 0.02); 
    if(!SmartDashboard.containsKey("VisionHold_Ki")) SmartDashboard.putNumber("VisionHold_Ki", 0.00); 
    if(!SmartDashboard.containsKey("VisionHold_Kd")) SmartDashboard.putNumber("VisionHold_Kd", 0.00);
    if(!SmartDashboard.containsKey("VisionHold_dz")) SmartDashboard.putNumber("VisionHold_dz", 0.3);  
  }

  public void setVoltages(double left, double right) {
    leftMotor1.setVoltage(left);
    rightMotor1.setVoltage(right);
  }

  public void setSpeed(double left, double right) {
    leftMotor1.set(left);
    rightMotor1.set(right);
  }

  public void setAutoAim() {
    leftMotor1.setIdleMode(IdleMode.kBrake);
    leftMotor2.setIdleMode(IdleMode.kBrake);
    rightMotor1.setIdleMode(IdleMode.kBrake);
    rightMotor2.setIdleMode(IdleMode.kBrake);
  }

  public double getLeftEncoderRevs() {
    return (leftEncoder1.getPosition() + leftEncoder2.getPosition()) / 2;
  }

  public double getLeftEncoderDistance() {
    return getLeftEncoderRevs() * WHEEL_CIRCUMFRENCE / GEAR_RATIO;
  }

  public double getRightEncoderRevs() {
    return (rightEncoder1.getPosition() + rightEncoder2.getPosition()) / 2;
  }

  public double getRightEncoderVelocity() {
    return ((rightEncoder1.getVelocity() + rightEncoder2.getVelocity()) / 2) * WHEEL_CIRCUMFRENCE  / (GEAR_RATIO *60);
  }

  public double getLeftEncoderVelocity() {
    return ((leftEncoder1.getVelocity() + leftEncoder2.getVelocity()) / 2) * WHEEL_CIRCUMFRENCE  / (GEAR_RATIO *60);
  }


  public double getRightEncoderDistance() {
    return getRightEncoderRevs() * WHEEL_CIRCUMFRENCE / GEAR_RATIO;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }

  private final ADIS16470_IMU gyro = new ADIS16470_IMU();

  public double getHeading() {
    return gyro.getAngle();
  }

  public void arcadeDrive(double throttle, double twist) {
    if (Math.abs(throttle) < 0.1) {
      throttle = 0;
    }
    if (Math.abs(twist) < 0.15) {
      twist = 0;
    }

    throttle *= Math.abs(throttle);
    twist *= Math.abs(twist);
    setVoltages(12 * (throttle + twist), 12 * (throttle - twist));
  }

  private final DifferentialDriveOdometry odometry =
      new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

  public void zeroHeading() {
    gyro.reset();
  }

  public void resetOdometry(Pose2d pose) {
    encoderReset();
    zeroHeading();
      odometry.resetPosition(pose, Rotation2d.fromDegrees(-getHeading()));
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
    pose = odometry.update(Rotation2d.fromDegrees(gyro.getAngle()), getLeftEncoderDistance(), getRightEncoderDistance());
    SmartDashboard.putNumber("Distance", (getLeftEncoderDistance() + getRightEncoderDistance())/2);
    Vision_kp = SmartDashboard.getNumber("VisionHold_Kp", 0.02);
    Vision_kp = SmartDashboard.getNumber("VisionHold_Ki", 0.0);
    Vision_kp = SmartDashboard.getNumber("VisionHold_Kd", 0.0);
    Vision_dz = SmartDashboard.getNumber("VisionHold_dz", 0.0);
  }

  public Pose2d getPose(){
    return pose;
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
