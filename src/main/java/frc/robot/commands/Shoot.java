// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Shoot extends SequentialCommandGroup {
  /** Creates a new Shoot. */
  public Shoot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoAim().until(() -> Math.abs(RobotContainer.shooter.getSpeed() - RobotContainer.shooter.getTarget()) < 2 
                            && Math.abs(RobotContainer.shooter.llTable.getEntry("tx").getDouble(0)) < 0.5).withTimeout(1)
                            .alongWith(new InstantCommand(RobotContainer.intake::intakeShoot, RobotContainer.intake))
                            .andThen((new InstantCommand(() -> RobotContainer.intake.setIntakeIn(0)))),
      new InstantCommand(() -> RobotContainer.drivetrain.setSpeed(0, 0)),
      new RunCommand(RobotContainer.transport::feederShoot, RobotContainer.transport)
    );
  }
}
