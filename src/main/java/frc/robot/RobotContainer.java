// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.drivers.EForwardableConnections;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Shooter.Mode;

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
  public static final PowerDistribution pdh = new PowerDistribution();
  SendableChooser<String> auton = new SendableChooser<>();

  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_Drivetrain);
  // dania stinks
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    portForwarding();
    configureButtonBindings();
    compressor.enableAnalog(60, 115);
    drivetrain.setDefaultCommand(new ArcadeDrive());
    transport.setDefaultCommand(new TransportIn());
    shooter.setDefaultCommand(new ShooterRampUpVoltage());
    intake.setDefaultCommand(new IntakeIn());
    SmartDashboard.putData("Auton Chooser", auton);
    if(SmartDashboard.getNumber("Climber Sequence", 0) == 0) {
      climber.setDefaultCommand(new DefaultClimberDown());
    }

    auton.addOption("TwoMeters", "TwoMeters");
    auton.addOption("RightBack", "RightBack");
    auton.addOption("TestArc", "TestArc");
    

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  public static final Joystick driverJoystick = new Joystick(0);
  private static final Joystick operatorJoystick = new Joystick(1);

  JoystickButton btn1 = new JoystickButton(driverJoystick, 1);
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

  JoystickButton opbtn1 = new JoystickButton(operatorJoystick, 1);
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
    btn1.whileHeld(new RunCommand(() -> transport.setSpeed(-0.5), transport).withTimeout(0.2).andThen(
      new RunCommand(transport::feederShoot, transport)))
        .whenReleased(
          new InstantCommand(transport::transportStop, transport).andThen(transport::clearBall, transport)
    );
    btn2.whileHeld(new AutoAim());
    btn3.toggleWhenPressed(
      new RunCommand(() -> transport.transportStop(), transport).alongWith(
        new RunCommand(() -> shooter.setSpeed(0), shooter), 
        new RunCommand(intake::stop, intake)
      ) 
    );
    btn4.whileHeld(new SetClimb());
    btn5.whileHeld(new SetMidRung());
    btn6.whileHeld(new SetExtend());
    btn7.whenPressed(new ToggleIntake().withTimeout(0.4));
    btn8.whileHeld(new Outtake());
    btn9.whenPressed(new ExpandClimberSol());
    btn10.whenPressed(new CompressClimberSol());
    btn11.whenPressed(new InstantCommand(() -> shooter.setMode(Mode.kFixedLow)));
    btn12.whenPressed(new InstantCommand(() -> shooter.setMode(Mode.kAuto))); 

    opbtn1.whenPressed(climber::incrementSequence);
    opbtn2.whenPressed(new InstantCommand(() -> shooter.setMode(Mode.kAuto)));
    opbtn3.whileHeld(new RunCommand(
      () -> {
      climber.setLeftLiftMotor(-0.35);
    }, climber)).whenReleased(new RunCommand(
      () -> {
      climber.setLeftLiftMotor(0.0);
    }, climber));
    opbtn4.whileHeld(new RunCommand(
      () -> {
      climber.setRightLiftMotor(-0.35);
    }, climber)).whenReleased(new RunCommand(
      () -> {
      climber.setRightLiftMotor(0.0);
    }, climber)); 
    opbtn5.whileHeld(new RunCommand(
      () -> {
      climber.setLeftLiftMotor(0.35);
    }, climber)).whenReleased(new RunCommand(
      () -> {
      climber.setLeftLiftMotor(0.0);
    }, climber));
    opbtn6.whileHeld(new RunCommand(
      () -> {
      climber.setRightLiftMotor(0.35);
    }, climber)).whenReleased(new RunCommand(
      () -> {
      climber.setRightLiftMotor(0.0);
    }, climber));
    opbtn8.whileHeld(new ClimbUp());
    opbtn7.whileHeld(new ClimbDown());
    opbtn9.whenPressed(new ExpandClimberSol());
    opbtn10.whenPressed(new CompressClimberSol());
    opbtn11.whenPressed(climber::resetSequence);
    opbtn12.whenPressed(new InstantCommand(() -> shooter.setMode(Mode.kFixedHigh)));

    // opbtn12.whileHeld(new SetMidRung());
    // opbtn11.whenPressed(new InstantCommand(
    //   () -> {
    //     climber.reset();
    //   }, climber));
    
    

    // opbtn11.whileHeld(new RunCommand(
    //   () -> {
    //   shooter.leftShooter.set(ControlMode.PercentOutput, 0.25);
    // }, shooter)).whenReleased(new RunCommand(
    //   () -> {
    //   shooter.leftShooter.set(ControlMode.PercentOutput, 0.0);
    // }, shooter));

    // opbtn12.whileHeld(new RunCommand(
    //   () -> {
    //   shooter.rightShooter.set(ControlMode.PercentOutput, 0.25);
    // }, shooter)).whenReleased(new RunCommand(
    //   () -> {
    //   shooter.rightShooter.set(ControlMode.PercentOutput, 0);
    // }, shooter));
  }

  public Command makeRamseteCommand(String path) {
    Path _path = Filesystem.getDeployDirectory().toPath().resolve("output/" + path + ".wpilib.json");
    try {
      RamseteCommand command = new RamseteCommand(
        TrajectoryUtil.fromPathweaverJson(_path), 
        drivetrain::getPose, 
        new RamseteController(Constants.DrivetrainConstants.kRamseteB, Constants.DrivetrainConstants.kRamseteZeta), 
        new SimpleMotorFeedforward(Constants.DrivetrainConstants.kS, Constants.DrivetrainConstants.kV, Constants.DrivetrainConstants.kA), 
        Constants.DrivetrainConstants.KINEMATICS, 
        drivetrain::getWheelSpeeds, 
        new PIDController(0, 0, 0), 
        new PIDController(0, 0, 0), 
        drivetrain::setVoltages,
        drivetrain);
      return command;
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    return null;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   * @throws IOException
   */
  public Command getAutonomousCommand() throws IOException {
    Path path = Filesystem.getDeployDirectory().toPath().resolve("output/" + "RightBack" + ".wpilib.json");
    Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(path);
    drivetrain.resetOdometry(trajectory.getInitialPose());
    var stuff = new RunCommand(() -> shooter.setVoltage(5.25), shooter).alongWith(
      new InstantCommand(() -> transport.feeder.set(ControlMode.PercentOutput, -0.8))
        .andThen(new ToggleIntake().withTimeout(0.4))
        .andThen(new InstantCommand(() -> shooter.setMode(Mode.kAuto)))
        .andThen(new InstantCommand(() -> RobotContainer.intake.setIntakeIn(0.5)))
        .andThen(makeRamseteCommand("RightBack"))
        .andThen(new AutoAim().withTimeout(1))
        .andThen(new RunCommand(transport::feederShoot, transport).withTimeout(2))
        .andThen(new InstantCommand(() -> transport.feeder.set(ControlMode.PercentOutput, -0.8)))
        .andThen(makeRamseteCommand("Right1"))
        .andThen(makeRamseteCommand("Right2"))
        .andThen(new ToggleIntake().withTimeout(0.4))
        .andThen(new AutoAim().withTimeout(1))
        .andThen(new RunCommand(transport::feederShoot, transport).withTimeout(2))
        .andThen(new TurnToAngle(drivetrain.getHeading() + 80).withTimeout(2))
        .andThen(new RunCommand(() -> drivetrain.setVoltages(-8, -8), drivetrain).withTimeout(0.75))
        .andThen(new InstantCommand(() -> drivetrain.setVoltages(0, 0), drivetrain)));

    return stuff;
    // new RunCommand(() -> shooter.setVoltage(5.25), shooter).alongWith(
    //   new InstantCommand(() -> transport.feeder.set(ControlMode.PercentOutput, -0.8))
    //     .andThen(new ToggleIntake().withTimeout(0.4))
    //     .andThen(new InstantCommand(() -> shooter.setMode(Mode.kAuto)))
    //     .andThen(new InstantCommand(() -> RobotContainer.intake.setIntakeIn(0.5)))
    //     .andThen(makeRamseteCommand("RightBack"))
    //     .andThen(new AutoAim().withTimeout(1))
    //     .andThen(new RunCommand(transport::feederShoot, transport).withTimeout(3))
    //     .andThen(new InstantCommand(() -> transport.feeder.set(ControlMode.PercentOutput, -0.8)))
    //     .andThen(makeRamseteCommand("Right1"))
    //     .andThen(makeRamseteCommand("Right2"))
    //     .andThen(new ToggleIntake().withTimeout(0.4))
    //     .andThen(new AutoAim().withTimeout(1))
    //     .andThen(new RunCommand(transport::feederShoot, transport).withTimeout(3)));

    
  }
  private void portForwarding() {
    EForwardableConnections.addPortForwarding(EForwardableConnections.LIMELIGHT_CAMERA_FEED);
    EForwardableConnections.addPortForwarding(EForwardableConnections.LIMELIGHT_WEB_VIEW);
  }
}
