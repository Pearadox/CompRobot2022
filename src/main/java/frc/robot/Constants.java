// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DrivetrainConstants {
    public static final double kS = 0.16734; // Volts
    public static final double kV = 2.6713; // Volt Seconds per Meter
    public static final double kA = 0.27882; // Volt Seconds Squared per Meter

    public static final double kPVel = 6.9201; // Volt Seconds per Meter

    public static final double TRACK_WIDTH = 0.5461; // Meters
    public static final DifferentialDriveKinematics KINEMATICS =
        new DifferentialDriveKinematics(TRACK_WIDTH);

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(6.0); //Meters

    public static final double DISTANCE_PER_REVOLUTION = WHEEL_DIAMETER * Math.PI;
    public static final double PULSES_PER_REVOLUTION = 42 * 5.6;
    public static final double DISTANCE_PER_PULSE = DISTANCE_PER_REVOLUTION / PULSES_PER_REVOLUTION;
    public static final double SECONDS_PER_MINUTE = 60.0d;
    public static final double GEAR_REDUCTION = 13.8;

    public static final double MAX_VELOCITY = 3.6;
    public static final double MAX_ACCEL = 3;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double GEAR_RATIO = 364d / 36d;

    public static final double WHEEL_CIRCUMFRENCE = 2 * Math.PI * (WHEEL_DIAMETER/2);

    public static final int FRONT_RIGHT_MOTOR = 20;
    public static final int BACK_RIGHT_MOTOR = 21;

    public static final int FRONT_LEFT_MOTOR = 18;
    public static final int BACK_LEFT_MOTOR = 19;
  }

  public static final class ClimberConstants {
    // public static final int LEFT_SOLENOID = 2; //2
    // public static final int RIGHT_SOLENOID = 1; //1
    public static final int LEFT_LIFT_MOTOR = 25;
    public static final int RIGHT_LIFT_MOTOR = 26;
    public static final int LEFT_FOR_SOLENOID = 3;
    public static final int LEFT_REV_SOLENOID = 2;
    public static final int RIGHT_FOR_SOLENOID = 6;
    public static final int RIGHT_REV_SOLENOID = 7;
  }

  public static final class TransportConstants {
    public static final int TOP_TRANSPORT_MOTOR = 23;
    public static final int BOT_TRANSPORT_MOTOR = 22;
    public static final int BETER_TRANSPORT_MOTOR = 32;
  }

  public static final class IntakeConstants {
    public static final int INTAKE_MOTOR = 24;
    public static final int INTAKE_FOR_SOLENOID = 5; 
    public static final int INTAKE_REV_SOLENOID = 4; 
  }

  public static final class ShooterConstants {
    public static final int LEFT_SHOOTER = 31;
    public static final int RIGHT_SHOOTER = 30;
    public static final double MAXPERCENT = 0.4;
  }
}
