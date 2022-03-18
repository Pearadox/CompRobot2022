// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter.Mode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RightAuton extends SequentialCommandGroup {
  /** Creates a new RightAuton. */
  public RightAuton() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // addCommands(
    //   new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(new Pose2d(7.2, 1.347, rotation)),
    //   new InstantCommand(() -> RobotContainer.shooter.setMode(Mode.kAuto)),
    //   new ToggleIntake().withTimeout(0.1),
    //   new InstantCommand(() -> RobotContainer.intake.setIntakeIn(0.5)),
    //   new RunCommand(() -> RobotContainer.drivetrain.setVoltages(4, 4), RobotContainer.drivetrain).withTimeout(1),
    //   new InstantCommand(() -> RobotContainer.drivetrain.setVoltages(0, 0)),
    //   // new InstantCommand(() -> RobotContainer.intake.stop()),
    //   // new ToggleIntake().withTimeout(0.4),
    //   // new WaitCommand(2),
    //   new AutoAim().withTimeout(1),
    //   new RunCommand(RobotContainer.transport::feederShoot, RobotContainer.transport).withTimeout(3)
    //   // new RunCommand(() -> RobotContainer.drivetrain.setVoltages(-4, -4), RobotContainer.drivetrain).withTimeout(0.5)
    // );
  }
}
