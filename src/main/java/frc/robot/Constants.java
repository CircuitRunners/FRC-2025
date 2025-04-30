// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

    public class ElevatorConstants {
        public static final int kMotorPort = 40;

        public static final int kJoystickPort = 0;

        public static final double kElevatorKp = 2.0;
        public static final double kElevatorKi = 0;
        public static final double kElevatorKd = 0;

        public static final double kElevatorkS = 0.0; // volts (V)
        public static final double kElevatorkG = 1.9004; // volts (V)
        public static final double kElevatorkV = 1.95; // volt per velocity (V/(m/s))
        public static final double kElevatorkA = 0.25; // volt per acceleration (V/(m/s²))

        public static final double kElevatorGearing = 5;
        public static final double kElevatorDrumRadius = Units.inchesToMeters(1.75);
        public static final double kCarriageMass = 9.45278617; // kg

        public static final double kSetpointMeters = 0.75;
        // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
        public static final double kMinElevatorHeightMeters = 0.0;
        public static final double kMaxElevatorHeightMeters = 2.0;

        // distance per pulse = (distance per revolution) / (pulses per revolution)
        //  = (Pi * D) / ppr
        public static final double kElevatorEncoderDistPerPulse = (2.0 * Math.PI * kElevatorDrumRadius / 42);
    }

    public class ArmConstants {
        public static final int kMotorPort = 50;
        public static final int kJoystickPort = 0;

        public static final String kArmPositionKey = "ArmPosition";
        public static final String kArmPKey = "ArmP";

        // The P gain for the PID controller that drives this arm.
        public static final double kDefaultArmKp = 50.0;
        public static final double kDefaultArmSetpointDegrees = 75.0;

        // distance per pulse = (angle per revolution) / (pulses per revolution)
        //  = (2 * PI rads) / (4096 pulses)
        public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 42;

        public static final double kArmReduction = 5.0 * 5.0 * (100.0/38.0);
        public static final double kArmMass = 3.45278617 ; // Kilograms
        public static final double kArmLength = Units.inchesToMeters(12);
        public static final double kMinAngleRads = Units.degreesToRadians(-75);
        public static final double kMaxAngleRads = Units.degreesToRadians(255);
    }
}


