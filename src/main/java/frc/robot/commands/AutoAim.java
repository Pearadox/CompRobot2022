// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.fasterxml.jackson.databind.cfg.ContextAttributes;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class AutoAim extends CommandBase {
  public NetworkTable llTable = NetworkTableInstance.getDefault().getTable("limelight");
  double lastError = 0;
  double error_sum = 0;
  
  /** Creates a new AutoAim. */
  public AutoAim() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    llTable.getEntry("ledMode").setNumber(3);
    RobotContainer.drivetrain.setAutoAim();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double kS = 0.25;
    double kP = 0.075;
    double error = llTable.getEntry("tx").getDouble(0);
    double targets = llTable.getEntry("tv").getDouble(0);
    if(targets == 0 ) {      
      RobotContainer.drivetrain.arcadeDrive(
        -RobotContainer.driverJoystick.getY(), RobotContainer.driverJoystick.getZ());
    }
    else {
      if(Math.abs(RobotContainer.driverJoystick.getY()) < 0.3) {
        if (Math.abs(error) > 0.5) {
          RobotContainer.drivetrain.setVoltages(kS * Math.signum(error) + kP * error + -RobotContainer.driverJoystick.getY(), -kP * error - kS * Math.signum(error) + -RobotContainer.driverJoystick.getY());
        } else {
          RobotContainer.drivetrain.setVoltages(0, 0);
        }
      }
      else {
        double changeinError = lastError - error;
        error_sum += error;
        double p = RobotContainer.drivetrain.Vision_kp * error;
        double i = RobotContainer.drivetrain.Vision_ki * error_sum;
        double d = RobotContainer.drivetrain.Vision_kd * changeinError;
        double speed = -RobotContainer.driverJoystick.getY();
        lastError = error;
        double output = p + i + d;
          RobotContainer.drivetrain.setSpeed(speed - output, speed + output);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    // llTable.getEntry("ledMode").setNumber(1);
    RobotContainer.drivetrain.setMode(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
