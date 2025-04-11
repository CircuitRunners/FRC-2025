package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.Date;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

// import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveRotation;
import com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveTranslation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
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
import frc.robot.Constants;
// import frc.robot.Vision;
// import frc.robot.Vision.VisionMeasurement;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.io.DriverControls;

import java.time.LocalDate;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;

public class Drive extends SubsystemBase {
  public static double limit = 1;
  private Swerve swerve;
  private FieldUtil fieldUtil = FieldUtil.getField();
  private boolean sysIdTranslator = true;
  public long setTime;
  
  // public Vision vision;
  public boolean visionRunning;
  
  private SwerveRequest.FieldCentric driveRequest;

  public Vision vision;
  public static final double kPX = 2;
  public static final double kPY = 2;
  public static final double kPTheta = 3;
  public static final double kXTolerance = 0.05;
  public static final double kYTolerance = 0.05;
  public static final double kThetaTolerance = 0.01;


  public static final double leftL4AlignmentX = 0.7;
  public static final double leftAlignmentX = 0.4445;
  public static final double leftL4ScoreAlignmentX = 0.5;
  public static final double leftAlignmentY = -0.17;


  public static final double rightL4AlignmentX = leftL4AlignmentX;
  public static final double rightAlignmentX = leftAlignmentX;
  public static final double rightL4ScoreAlignmentX = leftL4ScoreAlignmentX;

  public static final double rightAlignmentY = 0.15;
  
  public static final double thetaAlignment = 0;

  private PIDController xController = new PIDController(kPX, 0, 0.001);
  private PIDController yController = new PIDController(kPY, 0, 0.001);
  private PIDController thetaController = new PIDController(kPTheta, 0, 0.001);

  StructPublisher<Pose2d> finalPoseEstimate = NetworkTableInstance.getDefault().getStructTopic("SmartDashboard/Subsystem/Swerve/finalPoseEstimate", Pose2d.struct).publish();
  StructPublisher<ChassisSpeeds> curChassisSpeed = NetworkTableInstance.getDefault().getStructTopic("SmartDashboard/Subsystem/Swerve/curChassisSpeeds", ChassisSpeeds.struct).publish();
  StructArrayPublisher<SwerveModuleState> swerveModuleState = NetworkTableInstance.getDefault().getStructArrayTopic("SmartDashboard/Subsystem/Swerve/curModuleStates", SwerveModuleState.struct).publish();


/* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
  private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
      new SysIdRoutine.Config(
          null,        // Use default ramp rate (1 V/s)
          Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
          null,        // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
      ),
      new SysIdRoutine.Mechanism(
          output -> swerve.setControl(m_translationCharacterization.withVolts(output)),
          null,
          this
      )
  );

  /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
  private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
      new SysIdRoutine.Config(
          null,        // Use default ramp rate (1 V/s)
          Volts.of(7), // Use dynamic voltage of 7 V
          null,        // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
      ),
      new SysIdRoutine.Mechanism(
          volts -> swerve.setControl(m_steerCharacterization.withVolts(volts)),
          null,
          this
      )
  );

  private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                swerve.setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

  private SlewRateLimiter forwardLimiter, strafeLimiter;
  /** Creates a new Drive */
  public Drive(Swerve swerve, boolean visionRunning) {
    
    SignalLogger.setPath("logs/sysid/drive");
    this.swerve = swerve;

    forwardLimiter = new SlewRateLimiter(10, -10, 0);
    strafeLimiter = new SlewRateLimiter(10, -10, 0);

    xController.setTolerance(kXTolerance);
    yController.setTolerance(kYTolerance);
    thetaController.setTolerance(kThetaTolerance);
    thetaController.enableContinuousInput(-Math.PI/2, Math.PI/2);

    vision = new Vision();
  }

  @Override
  public void periodic() {
    fieldUtil.setSwerveRobotPose(swerve.getPose2d(), swerve.getModuleStates(), SwerveConstants.modulePositions);
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
    // swerve.seedFieldCentric();
    swerve.resetRotation(new Rotation2d(Math.toRadians(0)));
  }

  public void resetOrientationAuto() {
    swerve.resetRotation(new Rotation2d(Math.toRadians(180)));
  }

  public void resetRotation(Rotation2d rotation ) {
    swerve.resetRotation(rotation);
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

  /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
      return sysIdTranslator ? m_sysIdRoutineTranslation.quasistatic(direction) : m_sysIdRoutineRotation.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine
   * specified by {@link #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return sysIdTranslator ? m_sysIdRoutineTranslation.dynamic(direction) : m_sysIdRoutineRotation.dynamic(direction);
  }

  // public Command sysIdDynamic(Direction direction){
  //   return sysIdTranslator ? sysIdTranslation.dynamic(direction) : sysIdRotation.dynamic(direction);
  // }

  // public Command sysIdQuasistatic(Direction direction){
  //   return sysIdTranslator ? sysIdTranslation.quasistatic(direction) : sysIdRotation.quasistatic(direction);
  // }

  public Command toggleSysIdMode(){
    return Commands.runOnce(() -> sysIdTranslator = !sysIdTranslator);
  }

  public Command autoAlignCommand(boolean left, Supplier<Boolean> l4, Supplier<Boolean> l4Score) {
    return startRun(()->{
      // Set the target pose based on the alignment side
      double targetX;
      if (l4Score.get()) {
        targetX = left ? leftL4ScoreAlignmentX : rightL4ScoreAlignmentX;
      } else if (l4.get()) {
        targetX = left ? leftL4AlignmentX : rightL4AlignmentX;
      } else {
        targetX = left ? leftAlignmentX : rightAlignmentX;
      }
      double targetY = left ? leftAlignmentY : rightAlignmentY;
      double targetTheta = thetaAlignment;

      // Set the PID controllers' setpoints
      xController.setSetpoint(targetX);
      yController.setSetpoint(targetY);
      thetaController.setSetpoint(targetTheta);

    },() -> {
      // Get the latest vision measurement (robot pose in tag space)
      Optional<Pose2d> visionOpt = vision.getRobotInTagSpace(left);
      
      if (visionOpt.isPresent()) {
        Pose2d visionPose = visionOpt.get();
        
        
        // Calculate corrections using your PID controllers
        double xPower = xController.calculate(visionPose.getX());
        double yPower = yController.calculate(visionPose.getY());
        double thetaPower = thetaController.calculate(visionPose.getRotation().getRadians());
        
        // Publish debug values to SmartDashboard (optional)
        SmartDashboard.putNumber("AutoAlign/xError", xController.getError());
        SmartDashboard.putNumber("AutoAlign/yError", yController.getError());
        SmartDashboard.putNumber("AutoAlign/thetaError", thetaController.getError());
        SmartDashboard.putNumber("AutoAlign/xPower", xPower);
        SmartDashboard.putNumber("AutoAlign/yPower", yPower);
        SmartDashboard.putNumber("AutoAlign/thetaPower", thetaPower);
        
        // Command the drive: Here we send the PID outputs as chassis speeds
        driveRobotCentric(new ChassisSpeeds(
            Math.max(-2, Math.min(2, -xPower)), 
            Math.max(-2, Math.min(2, -yPower)), 
            thetaPower
        ));
      } else {
        // If no vision data is available, stop the robot
        driveRobotCentric(new ChassisSpeeds(0, 0, 0));
      }
    }).until(() -> {
      // Terminate when the vision measurements are within the set tolerances
      return xController.atSetpoint() &&
      yController.atSetpoint() &&
      thetaController.atSetpoint();
    });

    // swerve.setControl(SwerveRequest.PointWheelsAt);
  }


  public Command autoAlignCommand(boolean left, Supplier<Boolean> l4) {
    return startRun(()->{
      // Set the target pose based on the alignment side
      double targetX;
      if (l4.get()) {
        targetX = left ? leftL4AlignmentX : rightL4AlignmentX;
      } else {
        targetX = left ? leftAlignmentX : rightAlignmentX;
      }
      double targetY = left ? leftAlignmentY : rightAlignmentY;
      double targetTheta = thetaAlignment;

      // Set the PID controllers' setpoints
      xController.setSetpoint(targetX);
      yController.setSetpoint(targetY);
      thetaController.setSetpoint(targetTheta);

    },() -> {
      // Get the latest vision measurement (robot pose in tag space)
      Optional<Pose2d> visionOpt = vision.getRobotInTagSpace(left);
      
      if (visionOpt.isPresent()) {
        Pose2d visionPose = visionOpt.get();
        
        
        // Calculate corrections using your PID controllers
        double xPower = xController.calculate(visionPose.getX());
        double yPower = yController.calculate(visionPose.getY());
        double thetaPower = thetaController.calculate(visionPose.getRotation().getRadians());
        
        // Publish debug values to SmartDashboard (optional)
        SmartDashboard.putNumber("AutoAlign/xError", xController.getError());
        SmartDashboard.putNumber("AutoAlign/yError", yController.getError());
        SmartDashboard.putNumber("AutoAlign/thetaError", thetaController.getError());
        SmartDashboard.putNumber("AutoAlign/xPower", xPower);
        SmartDashboard.putNumber("AutoAlign/yPower", yPower);
        SmartDashboard.putNumber("AutoAlign/thetaPower", thetaPower);
        
        // Command the drive: Here we send the PID outputs as chassis speeds
        driveRobotCentric(new ChassisSpeeds(
            Math.max(-2, Math.min(2, -xPower)), 
            Math.max(-2, Math.min(2, -yPower)), 
            thetaPower
        ));
      } else {
        // If no vision data is available, stop the robot
        driveRobotCentric(new ChassisSpeeds(0, 0, 0));
      }
    }).until(() -> {
      // Terminate when the vision measurements are within the set tolerances
      return xController.atSetpoint() &&
      yController.atSetpoint() &&
      thetaController.atSetpoint();
    });

    // swerve.setControl(SwerveRequest.PointWheelsAt);
  }

  public Command autoAlignCommand(boolean left) {
    boolean l4 = true;
    return startRun(()->{
      // Set the target pose based on the alignment side
      double targetX;
      if (l4) {
        targetX = left ? leftL4AlignmentX : rightL4AlignmentX;
      } else {
        targetX = left ? leftAlignmentX : rightAlignmentX;
      }
      double targetY = left ? leftAlignmentY : rightAlignmentY;
      double targetTheta = thetaAlignment;

      // Set the PID controllers' setpoints
      xController.setSetpoint(targetX);
      yController.setSetpoint(targetY);
      thetaController.setSetpoint(targetTheta);

    },() -> {
      // Get the latest vision measurement (robot pose in tag space)
      Optional<Pose2d> visionOpt = vision.getRobotInTagSpace(left);
      
      if (visionOpt.isPresent()) {
        Pose2d visionPose = visionOpt.get();
        
        
        // Calculate corrections using your PID controllers
        double xPower = xController.calculate(visionPose.getX());
        double yPower = yController.calculate(visionPose.getY());
        double thetaPower = thetaController.calculate(visionPose.getRotation().getRadians());
        
        // Publish debug values to SmartDashboard (optional)
        SmartDashboard.putNumber("AutoAlign/xError", xController.getError());
        SmartDashboard.putNumber("AutoAlign/yError", yController.getError());
        SmartDashboard.putNumber("AutoAlign/thetaError", thetaController.getError());
        SmartDashboard.putNumber("AutoAlign/xPower", xPower);
        SmartDashboard.putNumber("AutoAlign/yPower", yPower);
        SmartDashboard.putNumber("AutoAlign/thetaPower", thetaPower);
        
        // Command the drive: Here we send the PID outputs as chassis speeds
        driveRobotCentric(new ChassisSpeeds(
            Math.max(-2, Math.min(2, -xPower)), 
            Math.max(-2, Math.min(2, -yPower)), 
            thetaPower
        ));
      } else {
        // If no vision data is available, stop the robot
        driveRobotCentric(new ChassisSpeeds(0, 0, 0));
      }
    }).until(() -> {
      // Terminate when the vision measurements are within the set tolerances
      return xController.atSetpoint() &&
      yController.atSetpoint() &&
      thetaController.atSetpoint();
    });
  }
}
