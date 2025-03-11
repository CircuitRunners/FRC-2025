// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.security.spec.EncodedKeySpec;

//Pathplanner
import com.pathplanner.lib.path.PathConstraints;
import com.ctre.phoenix6.StatusSignal;
import com.pathplanner.lib.config.PIDConstants;

//AprilTag
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

//Geometry
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Transform3d;

//Kinematics
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

//Units and stuff
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Distance;
//Swerves and utils
import frc.robot.generated.TunerConstants;
import swervelib.math.Matter;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class SwerveConstants{
    public static final int pigeonId = 1;
    public static final boolean inverGyro = true;

    /**
     * Drivetrain constants: must be tuned to specific robot
     */
    public final static double trackWidth = Units.inchesToMeters(28);
    public final static double wheelBase = Units.inchesToMeters(28);

    /**Swerve kinematics
     * Don't change unless you're not doing a 4 module drive
     */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      Mod0.position,
      Mod1.position,
      Mod2.position,
      Mod3.position);
    
    /** Module Gear Ratios */

    /**Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 25;
    public static final int drivePeakCurrentLimit = 40;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /** Angle Motor PID Values */
    public static final double angleKP = TunerConstants.steerGains.kP;
    public static final double angleKI = TunerConstants.steerGains.kI;
    public static final double angleKD = TunerConstants.steerGains.kD;
    public static final double angleKF = 0;

    /**Angle Motor SYSID values */
    public static final double angleKS = TunerConstants.steerGains.kS;
    public static final double angleKV = TunerConstants.steerGains.kV;
    public static final double angleKG = TunerConstants.steerGains.kG;
    public static final double angleKA = TunerConstants.steerGains.kA;

    /**Drive Motor PID Values */
    public static final double driveKP = TunerConstants.driveGains.kP;
    public static final double driveKI = TunerConstants.driveGains.kI;
    public static final double driveKD = TunerConstants.driveGains.kD;
    public static final double driveKF = 0;

    /** Drive Motor Characterization Values
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE*/
    public static final double driveKS = TunerConstants.driveGains.kS;
    public static final double driveKV = TunerConstants.driveGains.kV;
    public static final double driveKG = TunerConstants.driveGains.kG;
    public static final double driveKA = TunerConstants.driveGains.kA;

    /** Swerve Profiling Values
     * Meters per second
    */
    public static final double maxVelocityMPS = TunerConstants.kSpeedAt12Volts.baseUnitMagnitude();//Must be tuned to specific robot
    public static final double maxModuleVelocityMPS = maxVelocityMPS;
    public static final double maxModuleAccelerationMPSSq = 2.0; //Must be tuned to specific robot

    /**Radians per Second */
    public static final double maxAngularVelocityRPS = 10.0;//Must be tuned to specific robot
    public static final double maxAngularAccelerationRPSSq = 3.0;//Must be tuned to specific robot

    public static final double slipCurrent = TunerConstants.kSlipCurrent.magnitude();
    public static final double steerInertia = TunerConstants.kSteerInertia.baseUnitMagnitude();
    public static final double driveInertia = TunerConstants.kDriveInertia.baseUnitMagnitude();
    public static final double couplingGearRatio = TunerConstants.kCoupleRatio;
    public static final String CANBusName = TunerConstants.kCANBus.getName();
    public static final double driveBaseRadiusMeter = Units.inchesToMeters(12);
    public static final Translation2d[] modulePositions = { Mod0.position, Mod1.position, Mod2.position,
        Mod3.position };

    /**PID Constants */
    public static final PIDConstants translationalPID = new PIDConstants(driveKP, driveKI, driveKD);
    public static final PIDConstants rotationalPID = new PIDConstants(angleKP, angleKI, angleKD);

    /**Front Left Module - Module 0 */
    //Must be tuned to specific robot
    public static final class Mod0{
      public static final int driveMotorID = 0;
      public static final int angleMotorID = 1;
      public static final int canCoderID = 0;

      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(TunerConstants.kFrontLeftEncoderOffset.baseUnitMagnitude());
      public static final Translation2d position = new Translation2d(driveBaseRadiusMeter/2.0,wheelBase/2.0);//front left +x,+y
    }

    /**Front Right Module - Module 1 */
    //Must be tuned to specific robot
    public static final class Mod1 {
      public static final int driveMotorID = 2;
      public static final int angleMotorID = 3;
      public static final int canCoderID = 1;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(TunerConstants.kFrontRightEncoderOffset.baseUnitMagnitude());
      public static final Translation2d position = new Translation2d(driveBaseRadiusMeter / 2.0, -wheelBase / 2.0); //front right is +x, -y
    }
  
    /**Back Left Module - Module 2 */
    //Must be tuned to specific robot
    public static final class Mod2{
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 2;

      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(TunerConstants.kBackLeftEncoderOffset.baseUnitMagnitude());
      public static final Translation2d position = new Translation2d(driveBaseRadiusMeter/-2.0, wheelBase/2.0);//Back left -x,+y
    }
  
    /**Back Right Module - Module 3 */
    //Must be tuned to specific robot
    public static final class Mod3{
      public static final int driveMotorID = 6;
      public static final int angleMotorID = 7;
      public static final int canCoderID = 3;

      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(TunerConstants.kBackRightEncoderOffset.baseUnitMagnitude());
      public static final Translation2d position = new Translation2d(-driveBaseRadiusMeter/2.0,-wheelBase/2.0);// -x,-y
    }
  
    public static final PathConstraints pathConstraints = new PathConstraints(maxVelocityMPS, maxModuleAccelerationMPSSq, maxAngularVelocityRPS, maxAngularAccelerationRPSSq);
    public static final double robotMass = 54.431;
    public static final double MOI = 6.898;
    public static final double wheelCOF = 1.0;
  }

  public static final class DriverConstants{
    public static final double stickDeadband = 0.2;
    public static final int driverPort = 0;
    public static final int operatorPort = 1;
  }

  public static final class TestElevatorConstants {
    public static final int motor1port = 21;
    public static final int motor2port = 22;

    public static final double maxEncoderValue = 0; // tune specifically
    public static final double minEncoderValue = 0; // tune specifically
    public static final double l1EncoderValue = 0; // tune specifically
    public static final double l2EncoderValue = 0; // tune specifically
    public static final double l3EncoderValue = 0; // tune specifically
    public static final double l4EncoderValue = 0; // tune specifically

  }

  public static final class TestArmConstants {
    public static final int motor1port = 23;
    public static final int motor2port = 24;

    public static final double maxEncoderValue = 0; //currently set with Neo Encoder
    public static final double minEncoderValue = 0; //currently set with Neo Encoder
    public static final double horizontalEncoderValue = 0; //currently set with Neo Encoder
    public static final double l4EncoderValue = 0; //currently set with Neo Encoder
  }
  public static final class VisionConstants{
    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark); //IGNORE THE ERROR
    public static final Transform3d frontCamTransform = new Transform3d();
    public static final Transform3d backCamTransform = new Transform3d();
  }

   public class FieldConstants{
    public static final Distance kFieldLength = Meters.of(16.54);
    public static final Distance kFieldWidth = Meters.of(8.21);
    public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark); //IGNORE THE ERROR
   }

   public static class ElevatorConstants{
    public static final int elevatorPort1 = 21;
    public static final int elevatorPort2 = 22;
    
    
    public static double encoderDifference = 40.571014404296875; // tune specifically
    public static final double maxEncoderValue = -73.50224304199219 - encoderDifference; // tune specifically
    public static final double minEncoderValue = 0;
    public static final double l1EncoderValue = 2.857138395309448 - encoderDifference; // tune specifically
    public static final double l2EncoderValue = -10.238096714019775 - encoderDifference; // tune specifically
    public static final double l3EncoderValue = -42.928138732910156 - encoderDifference; // tune specifically
    public static final double l4EncoderValue = -74.09685516357422 - encoderDifference; // tune specifically
    public static final double Algae2 = -50 - encoderDifference;
    public static final double Algae1 = -20 - encoderDifference;
    public static final double tolerance = 0.5;
  }
  
    public static final class ClawConstants {
      public static final int moveMotorPort = 23;
      public static final int rollerMotorPort1 = 24;
      public static final int rollerMotorPort2 = 25; 
  
      public static double encoderOffset = -95.86046600341797 / 5; //currently set with Neo Encoder
      public static double maxEncoderValue = 95.86046600341797 / 5 - encoderOffset; //currently set with Neo Encoder
      public static double minEncoderValue = 0;
      public static double horizontalEncoderValue = -95.86046600341797 / 5 - encoderOffset; //currently set with Neo Encoder
      public static double l4EncoderValue = 50.81052780151367 / 5 - encoderOffset; //currently set with Neo Encoder
      public static double tolerance = 0.5;
      public static int canRangePort = 0;
      public static double algaeValue = 40 / 5 - encoderOffset;
      public static double coralSensorRange = 0.5; 
    }


// STUFF FOR YAGSL
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

}

