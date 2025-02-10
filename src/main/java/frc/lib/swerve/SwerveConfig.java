package frc.lib.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.SwerveConstants;
import frc.robot.io.DriverControls;
import frc.robot.subsystems.Drive;

public class SwerveConfig {
  //TODO: tune configs
  private static final Slot0Configs driveConfigs = new Slot0Configs() {
    {
      kP = SwerveConstants.driveKP;
      kI = SwerveConstants.driveKI;
      kD = SwerveConstants.driveKD;
      kS = SwerveConstants.driveKS;
      kV = SwerveConstants.driveKV;
      kG = SwerveConstants.driveKG;
      kA = SwerveConstants.driveKA;
    }
  };
  private static final Slot0Configs turnConfigs = new Slot0Configs() {
    {
      kP = SwerveConstants.angleKP;
      kI = SwerveConstants.angleKI;
      kD = SwerveConstants.angleKD;
      kS = SwerveConstants.angleKS;
      kV = SwerveConstants.angleKV;
      kG = SwerveConstants.angleKG;
      kA = SwerveConstants.angleKA;
    }
  };

  private static final LegacySwerveModuleConstantsFactory CONSTANTS_FACTORY = new LegacySwerveModuleConstantsFactory()
      .withDriveMotorGearRatio(SwerveConstants.driveGearRatio)
      .withSteerMotorGearRatio(SwerveConstants.angleGearRatio)
      .withWheelRadius(Units.metersToInches(SwerveConstants.wheelCircumference / (2 * Math.PI)))
      .withSlipCurrent(SwerveConstants.slipCurrent)
      .withSteerMotorGains(turnConfigs)
      .withDriveMotorGains(driveConfigs)
      .withSpeedAt12VoltsMps(SwerveConstants.maxVelocityMPS)
      .withSteerInertia(SwerveConstants.steerInertia)
      .withDriveInertia(SwerveConstants.driveInertia)
      .withSteerMotorInverted(false)
      .withCouplingGearRatio(SwerveConstants.couplingGearRatio);

  private static final LegacySwerveModuleConstants generateConstants(int turnID, int driveID, int canCoderID,
      Translation2d position, double absoluteOffset) {
    return CONSTANTS_FACTORY.createModuleConstants(turnID, driveID, canCoderID, absoluteOffset, position.getX(),
        position.getY(), false);

  }

  public static Swerve getConfiguredDrivetrain() {
    var drivetrain = new LegacySwerveDrivetrainConstants()
        .withPigeon2Id(SwerveConstants.pigeonId)
        .withCANbusName(SwerveConstants.CANBusName);
    var frontLeft = generateConstants(SwerveConstants.Mod0.angleMotorID, SwerveConstants.Mod0.driveMotorID,
        SwerveConstants.Mod0.canCoderID, SwerveConstants.Mod0.position, SwerveConstants.Mod0.angleOffset.getRadians());
    var frontRight = generateConstants(SwerveConstants.Mod1.angleMotorID, SwerveConstants.Mod1.driveMotorID,
        SwerveConstants.Mod1.canCoderID, SwerveConstants.Mod1.position, SwerveConstants.Mod1.angleOffset.getRadians());
    var backLeft = generateConstants(SwerveConstants.Mod2.angleMotorID, SwerveConstants.Mod2.driveMotorID,
        SwerveConstants.Mod2.canCoderID, SwerveConstants.Mod2.position, SwerveConstants.Mod2.angleOffset.getRadians());
    var backRight = generateConstants(SwerveConstants.Mod3.angleMotorID, SwerveConstants.Mod3.driveMotorID,
        SwerveConstants.Mod3.canCoderID, SwerveConstants.Mod3.position, SwerveConstants.Mod3.angleOffset.getRadians());

    return new Swerve(drivetrain,
        new LegacySwerveModuleConstants[] { frontLeft, frontRight, backLeft, backRight });
  }

  public static final LegacySwerveRequest.FieldCentric drive = new LegacySwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  public static final LegacySwerveRequest.RobotCentric robotCentric = new LegacySwerveRequest.RobotCentric();
  public static final LegacySwerveRequest.SwerveDriveBrake brake = new LegacySwerveRequest.SwerveDriveBrake();
  public static final LegacySwerveRequest.PointWheelsAt pointWheelsAt = new LegacySwerveRequest.PointWheelsAt();
  public static final LegacySwerveRequest.ApplyChassisSpeeds applyChassisSpeeds = new LegacySwerveRequest.ApplyChassisSpeeds();

  public static ChassisSpeeds toChassisSpeeds(DriverControls driverControls, Drive drive) {
    return new ChassisSpeeds(driverControls.driveForward(), driverControls.driveStrafe(),
            driverControls.driveRotation());
  }
}