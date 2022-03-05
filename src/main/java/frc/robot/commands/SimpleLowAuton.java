// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimpleLowAuton extends SequentialCommandGroup {
  /** Creates a new SimpleLowAuton. */
  public SimpleLowAuton() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RunCommand(() -> RobotContainer.shooter.setVoltage(3), RobotContainer.shooter).alongWith(
        new RunCommand(() -> RobotContainer.transport.feederShoot() , RobotContainer.transport)
      ).withTimeout(2),
      new RunCommand(() -> RobotContainer.drivetrain.setVoltages(3, 3), RobotContainer.drivetrain).withTimeout(2),
      new RunCommand(() -> RobotContainer.drivetrain.setVoltages(0, 0))
    );
  }
}
