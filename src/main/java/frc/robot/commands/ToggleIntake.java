// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ToggleIntake extends CommandBase {
  /** Creates a new ToggleIntake. */
  public ToggleIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(RobotContainer.intake.getIntakeValue() == Value.kReverse || RobotContainer.intake.getIntakeValue() == Value.kOff) {
      RobotContainer.intake.setSpeed(-1);
    } else {
      RobotContainer.intake.setSpeed(1);
    }
    RobotContainer.intake.intakeToggleSol();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
