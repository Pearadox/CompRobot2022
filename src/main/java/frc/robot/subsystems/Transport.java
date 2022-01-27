// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Transport extends SubsystemBase {
  private static Transport INSTANCE;

  /** Creates a new Transport. */
  public Transport() {}

  public void dashboard() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static Transport getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Transport();
    }
    return INSTANCE;
  }

}
