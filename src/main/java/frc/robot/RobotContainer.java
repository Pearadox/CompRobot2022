// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

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
  public static final PicoColorSensor colorSensor = new PicoColorSensor();
  SendableChooser<Command> auton = new SendableChooser<>();
  public static final SendableChooser<Boolean> toggleFlashlight = new SendableChooser<>();

  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_Drivetrain);
  // dania stinks
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() throws IOException {
    // Configure the button bindings
    portForwarding();
    loadTrajectories();
    configureButtonBindings();
    compressor.enableAnalog(60, 110);
    drivetrain.setDefaultCommand(new ArcadeDrive());
    shooter.setDefaultCommand(new ShooterRampUpVoltage());
    intake.setDefaultCommand(new IntakeIn());
    SmartDashboard.putData("Auton Chooser", auton);
    climber.setDefaultCommand(new DefaultClimberDown());

    SmartDashboard.putData("Toggle Flashlight", toggleFlashlight);
    toggleFlashlight.setDefaultOption("Off", false);
    toggleFlashlight.addOption("On", true);
    SmartDashboard.putNumber("Angle", 0);
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
    btn1.whileHeld( // new RunCommand(() -> transport.setSpeed(-0.5), transport).withTimeout(0.2).andThen(
      new Shoot())
        .whenReleased(
          new InstantCommand(transport::transportStop, transport).andThen(transport::clearBall, transport)
    );
    btn2.whenPressed(new TurnToAngle(SmartDashboard.getNumber("Angle", 0))
        .until(() -> Math.abs(drivetrain.getHeading()-SmartDashboard.getNumber("Angle", 0)) < 5));
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
    opbtn6.whileHeld(new RunCommand(
      () -> {
      climber.setLeftLiftMotor(0.35);
    }, climber)).whenReleased(new RunCommand(
      () -> {
      climber.setLeftLiftMotor(0.0);
    }, climber));
    opbtn5.whileHeld(new RunCommand(
      () -> {
      climber.setRightLiftMotor(0.35);
    }, climber)).whenReleased(new RunCommand(
      () -> {
      climber.setRightLiftMotor(0.0);
    }, climber)); 
    opbtn4.whileHeld(new RunCommand(
      () -> {
      climber.setLeftLiftMotor(-0.35);
    }, climber)).whenReleased(new RunCommand(
      () -> {
      climber.setLeftLiftMotor(0.0);
    }, climber));
    opbtn3.whileHeld(new RunCommand(
      () -> {
      climber.setRightLiftMotor(-0.35);
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

  public static Joystick getOperJoystick() {
    return operatorJoystick;
  }

  private static ArrayList<Trajectory> trajectories = new ArrayList<>();
  private static ArrayList<String> paths = new ArrayList<>();

  private void loadTrajectories() throws IOException {
    paths.add("RightBack");
    paths.add("Right1");
    paths.add("Right2");
    paths.add("Left1");
    paths.add("Right3");
    paths.add("Right2_0");
    paths.add("TwoBallRude1");
    paths.add("TwoBallRude2");
    paths.add("TwoBallRude3");
    paths.add("TwoBallRude4");
    paths.add("FiveBall1");
    paths.add("FiveBall2");
    paths.add("FiveBall3");
    paths.add("FiveBall4");

    for(String path : paths) {
      Path _path = Filesystem.getDeployDirectory().toPath().resolve("output/" + path + ".wpilib.json");
      trajectories.add(TrajectoryUtil.fromPathweaverJson(_path));
    }
  }

  public static Command makeRamseteCommand(String path) {
      RamseteCommand command = new RamseteCommand(
        trajectories.get(paths.indexOf(path)), 
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
  }

  public void makeAuton() {
    var RightThreeAuton = new InstantCommand(() -> shooter.setVoltage(5.37), shooter).andThen(
      new InstantCommand(() -> transport.feeder.set(ControlMode.PercentOutput, -0.8))
        .andThen(new ToggleIntake().withTimeout(0.5))
        .andThen(new InstantCommand(() -> shooter.setMode(Mode.kAuto)))
        .andThen(new InstantCommand(() -> RobotContainer.intake.setIntakeIn(0.5)))
        .andThen(makeRamseteCommand("RightBack"))
        .andThen(new ToggleIntake().withTimeout(0.5))
        .andThen(new AutoAim().withTimeout(1))
        .andThen(new RunCommand(transport::feederShoot, transport).withTimeout(2))
        .andThen(new InstantCommand(() -> transport.feeder.set(ControlMode.PercentOutput, -0.8)))
        .andThen(new ToggleIntake().withTimeout(0.5))
        .andThen(new InstantCommand(() -> intake.setIntakeIn(1)))
        .andThen(new InstantCommand(() -> shooter.setVoltage(5.2), shooter))
        .andThen(makeRamseteCommand("Right1"))
        .andThen(makeRamseteCommand("Right2"))
        .andThen(new ToggleIntake().withTimeout(0.4))
        .andThen(new AutoAim().withTimeout(1))
        .andThen(new RunCommand(transport::feederShoot, transport).withTimeout(2))
        .andThen(new InstantCommand(() -> drivetrain.setVoltages(0, 0), drivetrain)));
    
    var LeftThreeAuton = new RunCommand(() -> shooter.setVoltage(5.35), shooter).alongWith(
          new InstantCommand(() -> transport.feeder.set(ControlMode.PercentOutput, -0.8))
            .andThen(new ToggleIntake().withTimeout(0.5))
            .andThen(new InstantCommand(() -> shooter.setMode(Mode.kAuto)))
            .andThen(new InstantCommand(() -> RobotContainer.intake.setIntakeIn(0.5)))
            .andThen(makeRamseteCommand("RightBack"))
            .andThen(new AutoAim().withTimeout(1))
            .andThen(new RunCommand(transport::feederShoot, transport).withTimeout(2))
            .andThen(new InstantCommand(() -> transport.feeder.set(ControlMode.PercentOutput, -0.8)))
            .andThen(new edu.wpi.first.wpilibj2.command.WaitCommand(2))
            .andThen(makeRamseteCommand("Left1"))
            .andThen(makeRamseteCommand("Right2"))
            .andThen(new ToggleIntake().withTimeout(0.4))
            .andThen(new AutoAim().withTimeout(1))
            .andThen(new RunCommand(transport::feederShoot, transport).withTimeout(2))
            .andThen(new InstantCommand(() -> drivetrain.setVoltages(0, 0), drivetrain)));
    
    var TwoAuton = new RunCommand(() -> shooter.setVoltage(5.1), shooter).alongWith(
      new InstantCommand(() -> transport.feeder.set(ControlMode.PercentOutput, -0.8))
        .andThen(new ToggleIntake().withTimeout(0.4))
        .andThen(new InstantCommand(() -> shooter.setMode(Mode.kAuto)))
        .andThen(new InstantCommand(() -> RobotContainer.intake.setIntakeIn(0.5)))
        .andThen(makeRamseteCommand("RightBack"))
        .andThen(new ToggleIntake().withTimeout(0.5))
        .andThen(new AutoAim().withTimeout(1))
        .andThen(new RunCommand(transport::feederShoot, transport).withTimeout(3))
        .andThen(new InstantCommand(() -> transport.feeder.set(ControlMode.PercentOutput, -0.8))));
    
    var FourBall = new RunCommand(() -> shooter.setVoltage(5.02), shooter).alongWith(
      new InstantCommand(() -> transport.feeder.set(ControlMode.PercentOutput, -0.8))
        .andThen(new ToggleIntake().withTimeout(0.5))
        .andThen(new InstantCommand(() -> shooter.setMode(Mode.kAuto)))
        .andThen(new InstantCommand(() -> drivetrain.setVoltages(0, 0), drivetrain))
        .andThen(new InstantCommand(() -> RobotContainer.intake.setIntakeIn(0.4)))
        .andThen(makeRamseteCommand("RightBack"))
        // .andThen(new ToggleIntake().withTimeout(0.1))
        .andThen(new AutoAim().withTimeout(1.0))
        .andThen(new InstantCommand(() -> drivetrain.setVoltages(0, 0), drivetrain))
        .andThen(new RunCommand(transport::feederShoot, transport).withTimeout(1.5))
        // .andThen(new ToggleIntake().withTimeout(0.2))
        .andThen(new InstantCommand(() -> transport.feeder.set(ControlMode.PercentOutput, -0.8)))
        .andThen(new InstantCommand(() -> transport.setSpeed(0.4)))
        .andThen(makeRamseteCommand("Right1"))
        .andThen(makeRamseteCommand("Right2_0"))
        .andThen(makeRamseteCommand("Right3"))
        // .andThen(makeRamseteCommand("Right2-3"))
        .andThen(new RunCommand(() -> drivetrain.setVoltages(-8, -8), drivetrain).withTimeout(1.4))
        .andThen(new InstantCommand(() -> drivetrain.setVoltages(0, 0), drivetrain))
        .andThen(new ToggleIntake().withTimeout(0.3))
        .andThen(new AutoAim().withTimeout(1.0))
        .andThen(new InstantCommand(() -> drivetrain.setVoltages(0, 0), drivetrain))
        .andThen(new RunCommand(transport::feederShoot, transport).withTimeout(2)));

    var TwoBallRude = new InstantCommand(() -> shooter.setVoltage(5.25), shooter).andThen(
        new InstantCommand(() -> transport.feeder.set(ControlMode.PercentOutput, -0.8))
          .andThen(new ToggleIntake().withTimeout(0.5))
          .andThen(new InstantCommand(() -> shooter.setMode(Mode.kAuto)))
          .andThen(new InstantCommand(() -> drivetrain.setVoltages(0, 0), drivetrain))
          .andThen(new InstantCommand(() -> RobotContainer.intake.setIntakeIn(0.4)))
          .andThen(makeRamseteCommand("TwoBallRude1"))
          .andThen(new AutoAim().withTimeout(0.5))
          .andThen(new InstantCommand(() -> drivetrain.setVoltages(0, 0), drivetrain))
          .andThen(new RunCommand(transport::feederShoot, transport).withTimeout(2.5))
          .andThen(new InstantCommand(() -> transport.feeder.set(ControlMode.PercentOutput, -0.8)))
          .andThen(new InstantCommand(() -> transport.setSpeed(0.4)))
          .andThen(makeRamseteCommand("TwoBallRude2"))
          .andThen(makeRamseteCommand("TwoBallRude3"))
          .andThen(makeRamseteCommand("TwoBallRude4"))
          .andThen(new InstantCommand(() -> shooter.setVoltage(3.25), shooter))
          .andThen(new InstantCommand(() -> drivetrain.setVoltages(0, 0), drivetrain))
          .andThen(new RunCommand(transport::feederShoot, transport).withTimeout(1.5))
          .andThen(new InstantCommand(() -> transport.feeder.set(ControlMode.PercentOutput, -0.8)))
          .andThen(new InstantCommand(() -> transport.setSpeed(0.4)))
          // .andThen(new RunCommand(() -> drivetrain.setVoltages(-8, -8), drivetrain).withTimeout(1.4))
          // .andThen(new InstantCommand(() -> drivetrain.setVoltages(0, 0), drivetrain))
          // .andThen(new ToggleIntake().withTimeout(0.3))
          // .andThen(new AutoAim().withTimeout(1.25))
          // .andThen(new InstantCommand(() -> drivetrain.setVoltages(0, 0), drivetrain))
          // .andThen(new RunCommand(transport::feederShoot, transport).withTimeout(2))
          );

    var FiveBall = new InstantCommand(() -> shooter.setVoltage(4.76))
        .andThen(new InstantCommand(() -> transport.feeder.set(ControlMode.PercentOutput, -0.8)))
        .andThen(new InstantCommand(() -> transport.setBotSpeed(0.4)))
        .andThen(new ToggleIntake().withTimeout(0.1))
        .andThen(new InstantCommand(() -> intake.setIntakeIn(1)))
        .andThen(makeRamseteCommand("FiveBall1"))
        .andThen(new Shoot().withTimeout(2.5))
        .andThen(new InstantCommand(() -> transport.feeder.set(ControlMode.PercentOutput, -0.8)))
        .andThen(new InstantCommand(() -> transport.setSpeed(0.4)))
        .andThen(new InstantCommand(() -> drivetrain.setVoltages(0, 0)))
        .andThen(new ToggleIntake().withTimeout(0.1))
        .andThen(new InstantCommand(() -> intake.setIntakeIn(1)))
        .andThen(makeRamseteCommand("FiveBall2"))
        .andThen(new RunCommand(() -> drivetrain.setVoltages(0, 0)).withTimeout(1.25))
        .andThen(makeRamseteCommand("FiveBall3"))
        .andThen(new InstantCommand(() -> drivetrain.setVoltages(0, 0)))
        .andThen(new Shoot().withTimeout(2.5));

    
    auton.setDefaultOption("TwoBallAuton", TwoAuton);
    auton.addOption("RightThreeBallAuton", RightThreeAuton);
    auton.addOption("LeftThreeBallAuton", LeftThreeAuton);
    auton.addOption("FourBall", FourBall);
    auton.addOption("TwoBallRude", TwoBallRude);
    auton.addOption("FiveBall", FiveBall);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   * @throws IOException
   */
  public Command getAutonomousCommand() throws IOException {
    
    if (auton.getSelected().toString().equals("TwoBallRude")) {
      drivetrain.resetOdometry(trajectories.get(paths.indexOf("TwoBallRude1")).getInitialPose());
    } else if(auton.getSelected().toString().equals("FiveBall")) {
      drivetrain.resetOdometry(trajectories.get(paths.indexOf("FiveBall1")).getInitialPose());
    } else {
      drivetrain.resetOdometry(trajectories.get(paths.indexOf("RightBack")).getInitialPose());
    }
    
    // if(auton.getSelected().equals("TwoBallAuton")) {
    //   return TwoAuton;
    // } else if (auton.getSelected().equals("RightThreeBallAuton")){
    //   return RightThreeAuton;
    // } else if (auton.getSelected().equals("LeftThreeBallAuton")){
    //   return LeftThreeAuton;
    // } else if (auton.getSelected().equals("TwoBallRude")) {
    //   drivetrain.resetOdometry(trajectories.get(paths.indexOf("TwoBallRude1")).getInitialPose());
    //   return TwoBallRude;
    // } else if (auton.getSelected().equals("FiveBall")){
    //   drivetrain.resetOdometry(trajectories.get(paths.indexOf("FiveBall1")).getInitialPose());
    //   return FiveBall;
    // } else {
    //   return FourBall;
    // }
    return auton.getSelected();
  }
  private void portForwarding() {
    EForwardableConnections.addPortForwarding(EForwardableConnections.LIMELIGHT_CAMERA_FEED);
    EForwardableConnections.addPortForwarding(EForwardableConnections.LIMELIGHT_WEB_VIEW);
  }
}
