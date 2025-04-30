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

public class Constants {
    public class ElevatorConstants {
        public static final int kMotorPort = 0;

        public static final int kJoystickPort = 0;

        public static final double kElevatorKp = 8;
        public static final double kElevatorKi = 0;
        public static final double kElevatorKd = .25;

        public static final double kElevatorkS = 0.0; // volts (V)
        public static final double kElevatorkG = 0.0; // volts (V)
        public static final double kElevatorkV = 0.0; // volt per velocity (V/(m/s))
        public static final double kElevatorkA = 0.0; // volt per acceleration (V/(m/s²))

        public static final double kElevatorGearing = 10.0;
        public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
        public static final double kCarriageMass = 2.0; // kg

        public static final double kSetpointMeters = 0.75;
        // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
        public static final double kMinElevatorHeightMeters = 0.0;
        public static final double kMaxElevatorHeightMeters = 2.0;

        // distance per pulse = (distance per revolution) / (pulses per revolution)
        //  = (Pi * D) / ppr
        public static final double kElevatorEncoderDistPerPulse = 2.0 * Math.PI * kElevatorDrumRadius / 42;
    }

    public class ArmConstants {
        public static final int kMotorPort = 1;
        public static final int kJoystickPort = 0;

        public static final String kArmPositionKey = "ArmPosition";
        public static final String kArmPKey = "ArmP";

        // The P gain for the PID controller that drives this arm.
        public static final double kDefaultArmKp = 50.0;
        public static final double kDefaultArmSetpointDegrees = 75.0;

        // distance per pulse = (angle per revolution) / (pulses per revolution)
        //  = (2 * PI rads) / (4096 pulses)
        public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 42;

        public static final double kArmReduction = 200;
        public static final double kArmMass = 4.0; // Kilograms
        public static final double kArmLength = Units.inchesToMeters(30);
        public static final double kMinAngleRads = Units.degreesToRadians(-75);
        public static final double kMaxAngleRads = Units.degreesToRadians(255);
    }

    public static class VisionConstants {
        public static final String kCameraName = "YOUR CAMERA NAME";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }
}
