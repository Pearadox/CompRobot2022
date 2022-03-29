// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ClimberZero extends CommandBase {

  boolean leftZeroed = false;
  boolean rightZeroed = false;

  /** Creates a new ClimberZero. */
  public ClimberZero() {
    addRequirements(RobotContainer.climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftZeroed = false;
    rightZeroed = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!leftZeroed) {
      RobotContainer.climber.setLeftLiftMotor(0.25);
      if (RobotContainer.climber.leftLiftMotor.getSupplyCurrent() > 17.5) {
        RobotContainer.climber.setLeftLiftMotor(0);
        leftZeroed = true;
      }
    }
    if (!rightZeroed) {
      RobotContainer.climber.setRightLiftMotor(0.25);
      if (RobotContainer.climber.rightLiftMotor.getSupplyCurrent() > 17.5) {
        RobotContainer.climber.setRightLiftMotor(0);
        rightZeroed = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.climber.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (leftZeroed && rightZeroed) || RobotContainer.climber.getStopping();
  }
}
