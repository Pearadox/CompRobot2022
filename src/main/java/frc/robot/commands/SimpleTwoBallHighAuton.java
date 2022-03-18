// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter.Mode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimpleTwoBallHighAuton extends SequentialCommandGroup {
  /** Creates a new SimpleTwoBallHighAuton. */
  public SimpleTwoBallHighAuton() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ToggleIntake().withTimeout(0.4),
      new InstantCommand(() -> RobotContainer.intake.setIntakeIn(0.5)),
      new RunCommand(() -> RobotContainer.drivetrain.setVoltages(4, 4), RobotContainer.drivetrain).withTimeout(1),
      new InstantCommand(() -> RobotContainer.drivetrain.setVoltages(0, 0)),
      new InstantCommand(() -> RobotContainer.intake.stop()),
      new ToggleIntake().withTimeout(0.4),
      new InstantCommand(() -> RobotContainer.shooter.setMode(Mode.kAuto)),
      new WaitCommand(2),
      new AutoAim().withTimeout(1),
      new RunCommand(RobotContainer.transport::feederShoot, RobotContainer.transport).withTimeout(5)
    );
  }
}
