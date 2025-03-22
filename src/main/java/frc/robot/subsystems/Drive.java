package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.Date;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.CANrange;
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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;

public class Drive extends SubsystemBase {
  public static double limit = 0.7;
  private Swerve swerve;
  private FieldUtil fieldUtil = FieldUtil.getField();
  private boolean sysIdTranslator = true;
  public long setTime;
  
  public Vision vision;
  public boolean visionRunning;
  private SwerveRequest.FieldCentric driveRequest;

  public CANrange distSensor1 = new CANrange(SwerveConstants.distanceSensor1Port, "Drivebase");
  public CANrange distSensor2 = new CANrange(SwerveConstants.distanceSensor2Port, "Drivebase");

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
  public Drive(Swerve swerve, boolean visionRunning) {
    
    SignalLogger.setPath("logs/sysid/drive");
    if(visionRunning) {
      vision = new Vision();
    }
    this.swerve = swerve;

    forwardLimiter = new SlewRateLimiter(10, -10, 0);
    strafeLimiter = new SlewRateLimiter(10, -10, 0);

    this.visionRunning = visionRunning;
    // swerve.setPigeonOffset();
    // resetGyroCommand().execute();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (visionRunning) {
      EstimatedRobotPose[] poses = vision.run(swerve.getPigeon2().getYaw().getValueAsDouble());
      swerve.addVisionMeasurement(poses[0].estimatedPose.toPose2d(), poses[0].timestampSeconds);
      swerve.addVisionMeasurement(poses[1].estimatedPose.toPose2d(), poses[1].timestampSeconds);
      fieldUtil.setObjectGlobalPose("LeftPoseEstimate", poses[0].estimatedPose.toPose2d());
    }
    SmartDashboard.putNumber("pigeon angle", swerve.getPigeon2().getYaw().getValueAsDouble());
    SmartDashboard.putNumber("drive limit", limit);

    double distance1 = distSensor1.getDistance().getValueAsDouble();
    double distance2 = distSensor2.getDistance().getValueAsDouble();
    double average = (distance1 + distance2) / 2;
    SmartDashboard.putNumber("dist1", distance1);
    SmartDashboard.putNumber("dist2", distance2);
    SmartDashboard.putBoolean("Good to lift", average < 0.6);
    // TODO FIND THE PERFECT POSITION, THEN FIND THE +/- WE HAVE BASED ON THAT
    SmartDashboard.putBoolean("Good to score", average < 0.455 && average > 0.42);
    SmartDashboard.putBoolean("Head on", Math.abs(distance1 - distance2) < 0.02);
    String direction;
    if (distance1 - distance2 > 0.02) {
      direction = "right";
    } else if (distance2 - distance1 > 0.02) {
      direction = "left";
    } else {
      direction = "good";
    }
    SmartDashboard.putString("Direction To Turn", direction);
    


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

  public void resetDrivecommand(DriverControls driverControls) {
    SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withDeadband(DriverConstants.stickDeadband);
    this.setDefaultCommand(this.driveFieldCentricCommand(() -> 
      driveRequest
        .withVelocityX(driverControls.driveForward())
        .withVelocityY(driverControls.driveStrafe())
        .withRotationalRate(driverControls.driveRotation() * 0.8)  
    ));
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
  public void seedFieldCentric() {
    swerve.seedFieldCentric();
  }
  public void resetPose(Pose2d pose){
    swerve.resetPose(pose);
  }

  public void zeroGyro(double newYaw) {
    // swerve.getPigeon2().setYaw(newYaw);
  }

  public Command zeroGyroCommand(){
    return runOnce(() -> zeroGyro(0));
  }

  public Command driveToStrafeDistanceCommand(double dist, double speed) {
    return run(() -> driveRobotCentric(new ChassisSpeeds(0, speed, 0)))
        .until(() -> getPose().getTranslation().getDistance(new Translation2d(0, 0)) >
                     dist);
  }

  public Command driveToForwardDistanceCommand(double dist, double speed) {
    return run(() -> driveRobotCentric(new ChassisSpeeds(speed, 0, 0)))
        .until(() -> getPose().getTranslation().getDistance(new Translation2d(0, 0)) >
                     dist).finallyDo(this::brake);
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

  // public void addVisionMeasurement(){
  //   Consumer<VisionMeasurement> visionMeasurementConsumer = (visionMeasurement) -> {
  //     swerve.addVisionMeasurement(visionMeasurement.pose(), visionMeasurement.timestamp(), visionMeasurement.stdDev());
  //   };
  //   vision = new Vision(visionMeasurementConsumer);
  // }

}