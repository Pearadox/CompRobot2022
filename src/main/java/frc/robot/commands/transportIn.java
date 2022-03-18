// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class TransportIn extends CommandBase {
  /** Creates a new intakeIn. */
  double start;

  public TransportIn() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.transport);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    start = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.transport.transportIn();
    RobotContainer.transport.feederHold();
    if (Timer.getFPGATimestamp() - start > .5 && RobotContainer.transport.feeder.getSupplyCurrent() > 4.5) {
      RobotContainer.transport.detectBall();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.transport.transportStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
