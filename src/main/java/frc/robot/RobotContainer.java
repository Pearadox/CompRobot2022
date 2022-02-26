// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final Drivetrain drivetrain = Drivetrain.getInstance();
  public static final Intake intake = Intake.getInstance();
  public static final Transport transport = Transport.getInstance();
  public static final Shooter shooter = Shooter.getInstance();
  public static final Climber climber = Climber.getInstance();
  public static final Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);

  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_Drivetrain);
  // dania stinks
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    compressor.enableAnalog(60, 115);
    drivetrain.setDefaultCommand(new ArcadeDrive());
    transport.setDefaultCommand(new transportIn());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  public static final Joystick driverJoystick = new Joystick(0);
  private static final Joystick operatorJoystick = new Joystick(1);

  JoystickButton btn2 = new JoystickButton(driverJoystick, 2);
  JoystickButton btn3 = new JoystickButton(driverJoystick, 3);
  JoystickButton btn4 = new JoystickButton(driverJoystick, 4);
  JoystickButton btn5 = new JoystickButton(driverJoystick, 5);
  JoystickButton btn6 = new JoystickButton(driverJoystick, 6);
  JoystickButton btn7 = new JoystickButton(driverJoystick, 7);
  JoystickButton btn8 = new JoystickButton(driverJoystick, 8);
  JoystickButton btn9 = new JoystickButton(driverJoystick, 9);
  JoystickButton btn10 = new JoystickButton(driverJoystick, 10);
  JoystickButton btn11 = new JoystickButton(driverJoystick, 11);
  JoystickButton btn12 = new JoystickButton(driverJoystick, 12);

  JoystickButton opbtn2 = new JoystickButton(operatorJoystick, 2);
  JoystickButton opbtn3 = new JoystickButton(operatorJoystick, 3);
  JoystickButton opbtn4 = new JoystickButton(operatorJoystick, 4);
  JoystickButton opbtn5 = new JoystickButton(operatorJoystick, 5);
  JoystickButton opbtn6 = new JoystickButton(operatorJoystick, 6);
  JoystickButton opbtn7 = new JoystickButton(operatorJoystick, 7);
  JoystickButton opbtn8 = new JoystickButton(operatorJoystick, 8);
  JoystickButton opbtn9 = new JoystickButton(operatorJoystick, 9);
  JoystickButton opbtn10 = new JoystickButton(operatorJoystick, 10);
  JoystickButton opbtn11 = new JoystickButton(operatorJoystick, 11);
  JoystickButton opbtn12 = new JoystickButton(operatorJoystick, 12);

  private void configureButtonBindings() {
    btn2.whenPressed(new InstantCommand(intake::intakeOpenSol, intake));
    btn3.whenPressed(new compressClimberSol());
    btn4.whenPressed(new extendClimberSol());
    btn5.whileHeld(new climbUp());
    btn6.whileHeld(new climbDown());
    btn7.whileHeld(new intakeIn());
    btn8.whileHeld(new intakeOut());
    btn9.whileHeld(new transportIn());
    btn10.whileHeld(new transportOut());
    btn11.whileHeld(new ShooterRampUp());
    btn12.whenPressed(new ToggleIntake().withTimeout(0.25));
    opbtn7.whileHeld(new RunCommand(
      () -> {
      climber.setLeftLiftMotor(-0.35);
    }, climber)).whenReleased(new RunCommand(
      () -> {
      climber.setLeftLiftMotor(0.0);
    }, climber));
    opbtn8.whileHeld(new RunCommand(
      () -> {
      climber.setRightLiftMotor(-0.35);
    }, climber)).whenReleased(new RunCommand(
      () -> {
      climber.setRightLiftMotor(0.0);
    }, climber));

    opbtn9.whileHeld(new RunCommand(
      () -> {
      climber.setLeftLiftMotor(0.35);
    }, climber)).whenReleased(new RunCommand(
      () -> {
      climber.setLeftLiftMotor(0.0);
    }, climber));
    opbtn10.whileHeld(new RunCommand(
      () -> {
      climber.setRightLiftMotor(0.35);
    }, climber)).whenReleased(new RunCommand(
      () -> {
      climber.setRightLiftMotor(0.0);
    }, climber));

    opbtn11.whileHeld(new RunCommand(
      () -> {
      shooter.leftShooter.set(ControlMode.PercentOutput, 0.25);
    }, shooter)).whenReleased(new RunCommand(
      () -> {
      shooter.leftShooter.set(ControlMode.PercentOutput, 0.0);
    }, shooter));
    opbtn12.whileHeld(new RunCommand(
      () -> {
      shooter.rightShooter.set(ControlMode.PercentOutput, 0.25);
    }, shooter)).whenReleased(new RunCommand(
      () -> {
      shooter.rightShooter.set(ControlMode.PercentOutput, 0);
    }, shooter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
