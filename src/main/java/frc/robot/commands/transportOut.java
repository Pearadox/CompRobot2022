// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class TransportOut extends CommandBase {
  /** Creates a new intakeIn. */
  public TransportOut() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.transport);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.transport.transportOut();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.transport.transportStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(DriverStation.getAlliance() == Alliance.Blue){
      if(!SmartDashboard.getString("Color", "None").equals("Red")){
        return true;
      }
    }
    if(DriverStation.getAlliance() == Alliance.Red){
      if(!SmartDashboard.getString("Color", "None").equals("Blue")){
        return true;
      }
    }
    return false;
  }
}
