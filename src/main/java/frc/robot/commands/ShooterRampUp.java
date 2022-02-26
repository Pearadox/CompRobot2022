// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;

public class ShooterRampUp extends CommandBase {
  /** Creates a new ShooterRampUp. */
  public ShooterRampUp() {
    addRequirements(RobotContainer.shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  private double curPercentage;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    curPercentage = RobotContainer.shooter.getPercentOutput();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(curPercentage < SmartDashboard.getNumber("MaxPercentage", ShooterConstants.MAXPERCENT)) {
    //   RobotContainer.shooter.setSpeed(curPercentage);
    //   curPercentage += 0.01;
    // } else {
    //   RobotContainer.shooter.setSpeed(ShooterConstants.MAXPERCENT);
    // }
    RobotContainer.shooter.setSpeed(SmartDashboard.getNumber("MaxPercentage", ShooterConstants.MAXPERCENT));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
