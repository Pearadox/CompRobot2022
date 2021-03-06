// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.sql.Driver;

import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Outtake;
import frc.robot.commands.TransportIn;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;

  private Command m_autonomousCommand;

  public static int brownOutCtn = 0;

  // Add Debug flags
  // You can have a flag for each subsystem, etc
  public static final String _controls = "CONTROL";
  public static final String _general = "GENERAL";
  public static final String _auton = "AUTON";
  public static final String _drive = "DRIVE";
  public static final String drivetrain = "DRIVETRAIN";
  public static final String intake = "INTAKE";

  public SendableChooser<Boolean> llSwitch = new SendableChooser<>();

  public enum RobotState {
    DISABLED,
    AUTONOMOUS,
    TELEOP,
    TEST
  }

  public static RobotState s_robot_state = RobotState.DISABLED;

  public static RobotState getState() {
    return s_robot_state;
  }

  public static void setState(final RobotState state) {
    s_robot_state = state;
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    try {
      m_robotContainer = new RobotContainer();
    } catch (IOException e1) {
      // TODO Auto-generated catch block
      e1.printStackTrace();
    }

    RobotContainer.climber.flashlightOff();
    llSwitch.addOption("On", true);
    llSwitch.addOption("Off", false);
    llSwitch.setDefaultOption("On", true);
    SendableRegistry.setName(llSwitch, "Limelight Switch");
    SmartDashboard.putData(llSwitch);
    PortForwarder.add(8888, "limelight.local", 5800);
    PortForwarder.add(8889, "limelight.local", 5801);
    RobotContainer.shooter.setLeds(0);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Pressure", RobotContainer.compressor.getPressure());
    SmartDashboard.putNumber("Color Sensor Blue", RobotContainer.colorSensor.getRawColor0().blue);
    SmartDashboard.putNumber("Color Sensor Red", RobotContainer.colorSensor.getRawColor0().red);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    setState(RobotState.DISABLED);
    RobotContainer.drivetrain.setMode(false);
    RobotContainer.shooter.setLeds(0);
    RobotContainer.pdh.setSwitchableChannel(false);
  }

  @Override
  public void disabledPeriodic() {
    if(RobotContainer.drivetrain.leftMotor1.getIdleMode() == IdleMode.kBrake) {
      RobotContainer.drivetrain.setMode(false);
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    RobotContainer.drivetrain.setAutoAim();
    RobotContainer.shooter.setLeds(3);
    
    try {
      m_autonomousCommand = m_robotContainer.getAutonomousCommand();
  } catch (IOException e) {
    // TODO Auto-generated catch block
    e.printStackTrace();
  }

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    RobotContainer.transport.setDefaultCommand(new TransportIn());
    RobotContainer.drivetrain.setMode(true);
    RobotContainer.shooter.setLeds(3);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if(RobotContainer.toggleAutoReject.getSelected()){
    if(DriverStation.getAlliance() == Alliance.Red){
      if(SmartDashboard.getString("Color", "None").equals("Blue")){
        CommandScheduler.getInstance().schedule(new Outtake().withTimeout(1.5));
      }
    }
    if(DriverStation.getAlliance() == Alliance.Blue){
      if(SmartDashboard.getString("Color", "None").equals("Red")){
        CommandScheduler.getInstance().schedule(new Outtake().withTimeout(1.5));
      }
    }
  }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {
    REVPhysicsSim.getInstance().addSparkMax(m_robotContainer.drivetrain.leftMotor1, DCMotor.getNEO(1));
  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }
}
