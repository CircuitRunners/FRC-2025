package frc.robot.generated;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;

import frc.lib.swerve.Swerve;

// Generated by the Tuner X Swerve Project Generator
// https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
public class TunerConstants {
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    public static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.5)
        .withKS(0.1).withKV(2.66).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    public static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(0.1).withKI(0).withKD(0)
        .withKS(0).withKV(0.124);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    public static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    public static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The type of motor used for the drive motor
    public static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    // The type of motor used for the drive motor
    public static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
    public static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    public static final Current kSlipCurrent = Amps.of(120.0);

    // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    public static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    public static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                // Swerve azimuth does not require much torque output, so we can set a relatively low
                // stator current limit to help avoid brownouts without impacting performance.
                .withStatorCurrentLimit(Amps.of(60))
                .withStatorCurrentLimitEnable(true)
        );
    public static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    public static final Pigeon2Configuration pigeonConfigs = null;

    // CAN bus that the devices are located on;
    // All swerve devices must share the same CAN bus
    public static final CANBus kCANBus = new CANBus("Drivebase", "./logs/example.hoot");

    // Theoretical free speed (m/s) at 12 V applied output;
    // This needs to be tuned to your individual robot
    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.73);

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    public static final double kCoupleRatio = 3.5714285714285716;

    public static final double kDriveGearRatio = 6.746031746031747;
    public static final double kSteerGearRatio = 21.428571428571427;
    public static final Distance kWheelRadius = Inches.of(2);

    public static final boolean kInvertLeftSide = false;
    public static final boolean kInvertRightSide = true;

    public static final int kPigeonId = 0;

    // These are only used for simulation
    public static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
    public static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
    // Simulated voltage necessary to overcome friction
    public static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    public static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(kCANBus.getName())
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs);

    public static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withCouplingGearRatio(kCoupleRatio)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage);


    // Front Left
    public static final int kFrontLeftDriveMotorId = 0;
    public static final int kFrontLeftSteerMotorId = 10;
    public static final int kFrontLeftEncoderId = 0;
    public static final Angle kFrontLeftEncoderOffset = Rotations.of(0.055419921875);
    public static final boolean kFrontLeftSteerMotorInverted = true;
    public static final boolean kFrontLeftEncoderInverted = false;

    public static final Distance kFrontLeftXPos = Inches.of(14);
    public static final Distance kFrontLeftYPos = Inches.of(14);

    // Front Right
    public static final int kFrontRightDriveMotorId = 1;
    public static final int kFrontRightSteerMotorId = 11;
    public static final int kFrontRightEncoderId = 1;
    public static final Angle kFrontRightEncoderOffset = Rotations.of(0.497802734375);
    public static final boolean kFrontRightSteerMotorInverted = true;
    public static final boolean kFrontRightEncoderInverted = false;

    public static final Distance kFrontRightXPos = Inches.of(14);
    public static final Distance kFrontRightYPos = Inches.of(-14);

    // Back Left
    public static final int kBackLeftDriveMotorId = 2;
    public static final int kBackLeftSteerMotorId = 12;
    public static final int kBackLeftEncoderId = 2;
    public static final Angle kBackLeftEncoderOffset = Rotations.of(-0.25244140625);
    public static final boolean kBackLeftSteerMotorInverted = true;
    public static final boolean kBackLeftEncoderInverted = false;

    public static final Distance kBackLeftXPos = Inches.of(-14);
    public static final Distance kBackLeftYPos = Inches.of(14);

    // Back Right
    public static final int kBackRightDriveMotorId = 3;
    public static final int kBackRightSteerMotorId = 13;
    public static final int kBackRightEncoderId = 3;
    public static final Angle kBackRightEncoderOffset = Rotations.of(-0.076904296875);
    public static final boolean kBackRightSteerMotorInverted = true;
    public static final boolean kBackRightEncoderInverted = false;

    public static final Distance kBackRightXPos = Inches.of(-14);
    public static final Distance kBackRightYPos = Inches.of(-14);


    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
        ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
            kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
        ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
            kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
        ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
            kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted, kBackLeftEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
        ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
            kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted, kBackRightEncoderInverted
        );

    /**
     * Creates a Swerve instance.
     * This should only be called once in your robot program,.
     */
    public static Swerve createDrivetrain() {
        return new Swerve(
            DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight
        );
    }


    /**
     * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types.
     */
    public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
         * @param modules               Constants for each specific module
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, modules
            );
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency The frequency to run the odometry loop. If
         *                                unspecified or set to 0 Hz, this is 250 Hz on
         *                                CAN FD, and 100 Hz on CAN 2.0.
         * @param modules                 Constants for each specific module
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency, modules
            );
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
         *                                  unspecified or set to 0 Hz, this is 250 Hz on
         *                                  CAN FD, and 100 Hz on CAN 2.0.
         * @param odometryStandardDeviation The standard deviation for odometry calculation
         *                                  in the form [x, y, theta]ᵀ, with units in meters
         *                                  and radians
         * @param visionStandardDeviation   The standard deviation for vision calculation
         *                                  in the form [x, y, theta]ᵀ, with units in meters
         *                                  and radians
         * @param modules                   Constants for each specific module
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency,
                odometryStandardDeviation, visionStandardDeviation, modules
            );
        }
    }
}
