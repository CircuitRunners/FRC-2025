package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.Date;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveRotation;
import com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveTranslation;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.lib.swerve.Swerve;
import frc.lib.swerve.SwerveConfig;
import frc.lib.utils.FieldUtil;
import frc.lib.utils.PathPlannerUtil;
import frc.robot.Vision;
import frc.robot.Vision.VisionMeasurement;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.io.DriverControls;

import java.time.LocalDate;

public class Drive extends SubsystemBase {
  public static double limit = 0.7;
  private Swerve swerve;
  private FieldUtil fieldUtil = FieldUtil.getField();
  private boolean sysIdTranslator = true;
  public long setTime;
  
  private Vision vision;
  private boolean isVision;
  private SwerveRequest.FieldCentric driveRequest;
  // private final SysIdSwerveTranslation translation = new SysIdSwerveTranslation();
  // private final SysIdRoutine sysIdTranslation = new SysIdRoutine(
  //   new SysIdRoutine.Config(
  //     null, 
  //     Volts.of(7),
  //     null,
  //     null),  
  //   new SysIdRoutine.Mechanism(
  //     (volts) -> swerve.setControl(translation.withVolts(volts)),
  //     null,
  //     this)
  //   );
  // private final SysIdSwerveRotation rotation = new SysIdSwerveRotation();
  // private final SysIdRoutine sysIdRotation = new SysIdRoutine(
  //   new SysIdRoutine.Config(
  //     null,
  //     Volts.of(7),
  //     null,
  //     null),
  //   new SysIdRoutine.Mechanism(
  //     (volts) -> swerve.setControl(rotation.withVolts(volts)),
  //     null,
  //     this));
  private SlewRateLimiter forwardLimiter, strafeLimiter;
  /** Creates a new Drive */
  public Drive(Swerve swerve, boolean isVision) {
    
    SignalLogger.setPath("logs/sysid/drive");
    this.swerve = swerve;

    forwardLimiter = new SlewRateLimiter(10, -10, 0);
    strafeLimiter = new SlewRateLimiter(10, -10, 0);
    // swerve.setPigeonOffset();
    if (isVision) {
      addVisionMeasurement();
    }
    // resetGyroCommand().execute();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (isVision) {
      vision.run(swerve.getPigeon2().getYaw().getValueAsDouble());
    }
    SmartDashboard.putNumber("pigeon angle", swerve.getPigeon2().getYaw().getValueAsDouble() % 60);
    SmartDashboard.putNumber("drive limit", limit);
    fieldUtil.setSwerveRobotPose(swerve.getPose2d(), swerve.getModuleStates(), SwerveConstants.modulePositions);
    // SmartDashboard.putData("default pigeon value", swerve.getPigeon2().getRotation2d());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    Pose2d targetPose = PathPlannerUtil.getCurrentTargetPose();
    fieldUtil.setSwerveRobotPose(swerve.getPose2d(), swerve.getModuleStates(), SwerveConstants.modulePositions);
    fieldUtil.setObjectGlobalPose("Target Pose", targetPose);
    swerve.updateSimState(0.02, 12);
  }

  
  /** 
   * @param speeds
   */
  public void driveFieldCentric(SwerveRequest.FieldCentric driveRequest){
      swerve.setControl(driveRequest);
  }
    
    public void driveRobotCentric(ChassisSpeeds speeds){
      setTime = (new Date()).getTime();
      swerve.setControl(
        SwerveConfig.robotCentric
        .withVelocityX(forwardLimiter.calculate(speeds.vxMetersPerSecond))
        .withVelocityY(strafeLimiter.calculate(speeds.vyMetersPerSecond))
        // .withVelocityX(speeds.vxMetersPerSecond)
        // .withVelocityY(speeds.vyMetersPerSecond)
      .withRotationalRate(speeds.omegaRadiansPerSecond * 0.5)
    );
  }

  public void brake(){
    swerve.setControl(SwerveConfig.brake);
  }

  public Rotation2d geRotation2d(){
    return swerve.getPose2d().getRotation();
  }

  public Command driveFieldCentricCommand(Supplier<SwerveRequest.FieldCentric> driverRequest){
    return run(() -> driveFieldCentric(driverRequest.get()));
  }

  public Command driveRobotCentricCommand(Supplier<ChassisSpeeds> chassisSpeeds){
    return run(() -> driveRobotCentric(chassisSpeeds.get()));
  }

  public Command setLimitCommand(double newLimit) {
    return runOnce(() -> limit = newLimit);
  }

  public Command increaseLimitCommand() {
    return setLimitCommand(limit+0.2);
  }

  public Command decreaseLimitCommand() {
    return setLimitCommand(limit-0.2);
  }

  public ChassisSpeeds getChassisSpeeds(){
    return swerve.getState().Speeds;
  }

  public Command brakeCommand(){
    return runOnce(this::brake);
  }

  public Pose2d getPose(){
    return swerve.getPose2d();
  }

  public void resetPose(Pose2d pose){
    swerve.resetPose(pose);
  }

  public void zeroGyro() {
    swerve.getPigeon2().reset();
  }

  public Command zeroGyroCommand(){
    return runOnce(this::zeroGyro);
  }

  // public Command sysIdDynamic(Direction direction){
  //   return sysIdTranslator ? sysIdTranslation.dynamic(direction) : sysIdRotation.dynamic(direction);
  // }

  // public Command sysIdQuasistatic(Direction direction){
  //   return sysIdTranslator ? sysIdTranslation.quasistatic(direction) : sysIdRotation.quasistatic(direction);
  // }

  // public Command toggleSysIdMode(){
  //   return Commands.runOnce(() -> sysIdTranslator = !sysIdTranslator);
  // }

  // public void targetAngleDrive(Translation2d targetAngle, DriverControls controls){
  //   swerve.targetAngleDrive(targetAngle, controls.driveForward(), controls.driveStrafe());
  // }

  // public void targetAngleDrive(Rotation2d targetAngle, DriverControls controls){
  //   swerve.targetAngleDrive(targetAngle, controls.driveForward(), controls.driveStrafe());
  // }

  public void addVisionMeasurement(){
    Consumer<VisionMeasurement> visionMeasurementConsumer = (visionMeasurement) -> {
      swerve.addVisionMeasurement(visionMeasurement.pose(), visionMeasurement.timestamp(), visionMeasurement.stdDev());
    };
    vision = new Vision(visionMeasurementConsumer);
  }

}