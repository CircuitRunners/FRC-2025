package frc.lib.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.PointWheelsAt;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;
import com.pathplanner.lib.config.PIDConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.DeviceConstructor;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.ClawConstants.SwerveConstants;
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

  public static final PIDConstants translationPID = new PIDConstants(
    SwerveConstants.driveKP, 
    SwerveConstants.driveKI, 
    SwerveConstants.driveKD
  );

  public static final PIDConstants rotationPID = new PIDConstants(
    SwerveConstants.angleKP, 
    SwerveConstants.angleKI, 
    SwerveConstants.angleKD
  );
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

  private static final SwerveModuleConstantsFactory<TalonFXConfiguration,TalonFXConfiguration,CANcoderConfiguration> CONSTANTS_FACTORY = new SwerveModuleConstantsFactory<TalonFXConfiguration,TalonFXConfiguration,CANcoderConfiguration>()
      .withDriveMotorGearRatio(SwerveConstants.driveGearRatio)
      .withSteerMotorGearRatio(SwerveConstants.angleGearRatio)
      .withWheelRadius(Units.metersToInches(SwerveConstants.wheelCircumference / (2 * Math.PI)))
      .withSlipCurrent(SwerveConstants.slipCurrent)
      .withSteerMotorGains(turnConfigs)
      .withDriveMotorGains(driveConfigs)
      .withSpeedAt12Volts(SwerveConstants.maxVelocityMPS)
      .withSteerInertia(SwerveConstants.steerInertia)
      .withDriveInertia(SwerveConstants.driveInertia)
      .withCouplingGearRatio(SwerveConstants.couplingGearRatio);

  private static final SwerveModuleConstants<TalonFXConfiguration,TalonFXConfiguration, CANcoderConfiguration> generateConstants(int turnID, int driveID, int canCoderID,
      Translation2d position, double absoluteOffset) {
    return CONSTANTS_FACTORY.createModuleConstants(turnID, driveID, canCoderID, absoluteOffset, position.getX(),
        position.getY(), false, false, false);

  }

  // public static Swerve getConfiguredDrivetrain() {
  //   var drivetrain = new SwerveDrivetrainConstants()
  //       .withPigeon2Id(SwerveConstants.pigeonId)
  //       .withCANBusName(SwerveConstants.CANBusName);
  //   var frontLeft = generateConstants(SwerveConstants.Mod0.angleMotorID, SwerveConstants.Mod0.driveMotorID,
  //       SwerveConstants.Mod0.canCoderID, SwerveConstants.Mod0.position, SwerveConstants.Mod0.angleOffset.getRadians());
  //   var frontRight = generateConstants(SwerveConstants.Mod1.angleMotorID, SwerveConstants.Mod1.driveMotorID,
  //       SwerveConstants.Mod1.canCoderID, SwerveConstants.Mod1.position, SwerveConstants.Mod1.angleOffset.getRadians());
  //   var backLeft = generateConstants(SwerveConstants.Mod2.angleMotorID, SwerveConstants.Mod2.driveMotorID,
  //       SwerveConstants.Mod2.canCoderID, SwerveConstants.Mod2.position, SwerveConstants.Mod2.angleOffset.getRadians());
  //   var backRight = generateConstants(SwerveConstants.Mod3.angleMotorID, SwerveConstants.Mod3.driveMotorID,
  //       SwerveConstants.Mod3.canCoderID, SwerveConstants.Mod3.position, SwerveConstants.Mod3.angleOffset.getRadians());
  //   DeviceConstructor<TalonFX> driveMotorConstructor = new DeviceConstructor<TalonFX>() {
      
  //   };
  //   return new Swerve(null, null, null, null, frontLeft, frontRight, backLeft, backRight);
  // }

  public static final FieldCentric drive = new FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  public static final RobotCentric robotCentric = new RobotCentric();
  public static final SwerveDriveBrake brake = new SwerveDriveBrake();
  public static final PointWheelsAt pointWheelsAt = new PointWheelsAt();
  public static final ApplyRobotSpeeds applyChassisSpeeds = new ApplyRobotSpeeds();

  public static ChassisSpeeds toChassisSpeeds(DriverControls driverControls) {
    return new ChassisSpeeds(driverControls.driveForward(), driverControls.driveStrafe(),
            driverControls.driveRotation());
  }
}