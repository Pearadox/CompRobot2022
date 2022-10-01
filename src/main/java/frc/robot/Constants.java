// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

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
  public static final int TIMEOUT = 30;

  public static final class DrivetrainConstants {
    public static final double kS = 0.18579; // Volts
    public static final double kV = 3.2961; // Volt Seconds per Meter
    public static final double kA = 0.42341; // Volt Seconds Squared per Meter

    public static final double kPVel = 4.134; // Volt Seconds per Meter

    public static final double TRACK_WIDTH = Units.inchesToMeters(22.5); // Meters
    public static final DifferentialDriveKinematics KINEMATICS =
        new DifferentialDriveKinematics(TRACK_WIDTH);

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0); //Meters

    public static final double DISTANCE_PER_REVOLUTION = WHEEL_DIAMETER * Math.PI;
    public static final double PULSES_PER_REVOLUTION = 42 * 5.6;
    public static final double DISTANCE_PER_PULSE = DISTANCE_PER_REVOLUTION / PULSES_PER_REVOLUTION;
    public static final double SECONDS_PER_MINUTE = 60.0d;
    public static final double GEAR_REDUCTION = 13.8;

    public static final double MAX_VELOCITY = 3.6;
    public static final double MAX_ACCEL = 3;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double GEAR_RATIO = 1800d / 216d;
    //Hello World!
    // Hey Stem Savvy 


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
    public static final int LEFT_FIRST_EXTEND = 164904; //169000
    public static final int LEFT_EXTEND = -169000;
    public static final int LEFT_MID = -157000; //-157000
    public static final int LEFT_CLIMB = -11000;
    public static final int RIGHT_FIRST_EXTEND = 164904; //169000
    public static final int RIGHT_EXTEND = 169000; //169000
    public static final int RIGHT_MID = 157000;
    public static final int RIGHT_CLIMB = 11000;
    public static boolean kLeftSensorPhase = true;
    public static boolean kLeftMotorInvert = false;
    public static boolean kRightSensorPhase = true;
    public static boolean kRightMotorInvert = false;
    public static final int kPIDLoopIdx = 0;
    public static final int kSlotIdx = 0;
    public static final double kP = 30d / 2048;
    public static final double FEXTEND = 40 * 2048 * 15 / 9d;
    public static final double EXTEND = 50 * 2048 * 15 / 9d;
    public static final double MID = 21 * 2048 * 15 / 9d;
  }

  public static final class TransportConstants {
    public static final int TOP_TRANSPORT_MOTOR = 23;
    public static final int BOT_TRANSPORT_MOTOR = 22;
    public static final int BETER_TRANSPORT_MOTOR = 32;
  }

  public static final class IntakeConstants {
    public static final int INTAKE_RIGHT_MOTOR = 24;
    public static final int INTAKE_LEFT_MOTOR = 33;
    public static final int INTAKE_FOR_SOLENOID = 5; 
    public static final int INTAKE_REV_SOLENOID = 4; 
  }

  public static final class ShooterConstants {
    public static final int LEFT_SHOOTER = 31;
    public static final int RIGHT_SHOOTER = 30;
    public static final double MAXPERCENT = 0.4;
    public static final double ShooterAdjust = 1.125; //1.165

    public static final double kS = 0.69004; //0.69004 0.84535
    public static final double kV = 0.10758; //0.11811
    public static final double kP = 0.02; //0.04257 and 0.0307
  }

  public static int kTimeoutMs = 200;
}
