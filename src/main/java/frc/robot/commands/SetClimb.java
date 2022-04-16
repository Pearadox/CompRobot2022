// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SetClimb extends CommandBase {
  double initialTime;

  /** Creates a new SetClimb. */
  public SetClimb() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Timer.getFPGATimestamp() - initialTime < 1.0) {
      RobotContainer.climber.setClimbMotor(-0.65);
    } else {
      RobotContainer.climber.setClimbMotor(-1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    RobotContainer.climber.setLeftLiftMotor(0);
    RobotContainer.climber.setRightLiftMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return RobotContainer.climber.getLeftLiftEncoder() > 0 && RobotContainer.climber.getRightLiftEncoder() > 0;
    return RobotContainer.climber.getStopping();
  }
}
