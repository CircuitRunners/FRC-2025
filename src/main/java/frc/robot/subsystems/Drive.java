package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.Date;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveRotation;
import com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveTranslation;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

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
import frc.lib.LimelightHelpers.PoseEstimate;
import frc.lib.swerve.Swerve;
import frc.lib.swerve.SwerveConfig;
import frc.lib.utils.FieldUtil;
import frc.lib.utils.PathPlannerUtil;
import frc.robot.Vision;
import frc.robot.Vision.VisionMeasurement;
import frc.robot.Constants.SwerveConstants;
import frc.robot.io.DriverControls;

import java.time.LocalDate;

public class Drive extends SubsystemBase {
  public static double limit = 0.8;
  private Swerve swerve;
  private FieldUtil fieldUtil = FieldUtil.getField();
  private boolean sysIdTranslator = true;
  public long setTime;


  public static Pose2d reef1 = new Pose2d(6.113, 3.986, Rotation2d.fromDegrees(180)); 
  public static Pose2d reef2 = new Pose2d(5.304, 2.631, Rotation2d.fromDegrees(210)); 
  public static Pose2d reef3 = new Pose2d(3.646, 2.582, Rotation2d.fromDegrees(240)); 
  public static Pose2d reef4 = new Pose2d(2.195, 4.015, Rotation2d.fromDegrees(0)); 
  public static Pose2d reef5 = new Pose2d(3.686, 5.429, Rotation2d.fromDegrees(30)); 
  public static Pose2d reef6 = new Pose2d(5.353, 5.429, Rotation2d.fromDegrees(60)); 

  public static Pose2d[] poses = {reef1, reef2, reef3, reef4, reef5, reef6};
  
  private Vision vision;
  private boolean isVision;
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
    // if (isVision) {
    //   // addVisionMeasurement();
    // }
    vision = new Vision((visionMeasurement) -> {});
    resetToVisionPos();
  }
  
  public void resetToVisionPos() {
    PoseEstimate poseEstimate = vision.run(swerve.getPigeon2().getRotation2d());
    swerve.resetPose(poseEstimate.pose);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    PoseEstimate poseEstimate = vision.run(swerve.getPigeon2().getRotation2d());
    // swerve.addVisionMeasurement(visionMeasurement.pose(), visionMeasurement.timestamp(), visionMeasurement.stdDev());
    swerve.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);  
    // addVisionMeasurement();
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
  public void driveFieldCentric(ChassisSpeeds speeds){
    swerve.setControl(
      SwerveConfig.drive
      .withVelocityX(forwardLimiter.calculate(speeds.vxMetersPerSecond))
      .withVelocityY(strafeLimiter.calculate(speeds.vyMetersPerSecond))
      // .withVelocityX(speeds.vxMetersPerSecond)
      // .withVelocityY(speeds.vyMetersPerSecond)
      .withRotationalRate(speeds.omegaRadiansPerSecond * 0.5)
      );
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

  public Command driveFieldCentricCommand(Supplier<ChassisSpeeds> chassisSpeeds){
    return run(() -> driveFieldCentric(chassisSpeeds.get()));
  }

  public Command driveRobotCentricCommand(Supplier<ChassisSpeeds> chassisSpeeds){
    return run(() -> driveRobotCentric(chassisSpeeds.get()));
  }

  public Command setLimitCommand(double newLimit) {
    return runOnce(() -> limit = newLimit);
  }

  public ChassisSpeeds getChassisSpeeds(){
    return swerve.getState().Speeds;
  }

  public Command brakeCommand(){
    return run(this::brake);
  }

  public Pose2d getPose(){
    return swerve.getPose2d();
  }

  public void resetPose(Pose2d pose){
    swerve.resetPose(pose);
  }

  public Command resetGyroCommand(){
    return run(()->swerve.getPigeon2().reset());
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

  public Command returnFollowCommand() {

    var currentPose = this.getPose();
    double closestDistance = reef1.getTranslation().getDistance(currentPose.getTranslation());
    Pose2d closestPose = reef1;
    for (int i = 1; i < 6; i++) {
        if (currentPose.getTranslation().getDistance(poses[i].getTranslation()) < closestDistance) {
          closestDistance = currentPose.getTranslation().getDistance(poses[i].getTranslation());
          closestPose = poses[i];
        }
    }
    //  "globalConstraints": 
  //   "maxVelocity": 3.0,
  //   "maxAcceleration": 3.0,
  //   "maxAngularVelocity": 540.0,
  //   "maxAngularAcceleration": 720.0,
  //   "nominalVoltage": 12.0,
  //   "unlimited": false
  //
    return AutoBuilder.pathfindToPose(closestPose, new PathConstraints(3.0, 3.0, 540.0,720,12.0,false));
  }

}