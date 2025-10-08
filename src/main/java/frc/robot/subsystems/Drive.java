package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.Date;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

// import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveRotation;
import com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveTranslation;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
import frc.robot.subsystems.Vision.FieldPositions;

import java.time.LocalDate;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.epilogue.Logged;

@Logged
public class Drive extends SubsystemBase {
  public static double limit = 1;
  public Swerve swerve;
  private FieldUtil fieldUtil = FieldUtil.getField();
  private boolean sysIdTranslator = true;
  public long setTime;
  
  // public Vision vision;
  public boolean visionRunning;

  public Vision vision;
  public static final double kPX = 2;
  public static final double kPY = 2;
  public static final double kPTheta = 3;
  public static final double kXTolerance = 0.06;
  public static final double kYTolerance = 0.06;
  public static final double kThetaTolerance = 0.02;


  public static final double leftL4AlignmentX = 0.7;
  public static final double leftAlignmentX = 0.4045;
  public static final double leftL4ScoreAlignmentX = 0.46;   
  public static final double leftAlignmentY = -0.17;


  public static final double rightL4AlignmentX = leftL4AlignmentX;
  public static final double rightAlignmentX = leftAlignmentX;
  public static final double rightL4ScoreAlignmentX = leftL4ScoreAlignmentX;
  public static final double rightAlignmentY = 0.16;

  public static final double thetaAlignment = 0;

  public static final double kHPXTolerance = 0.05;
  public static final double kHPYTolerance = 0.05;
  public static final double kHPThetaTolerance = 0.05;


  public static final double leftHPAlignmentX = 3.5;
  public static final double leftHPAlignmentY = 1.41;
  public static final double leftHPThetaAlignment = Math.toRadians(6);
  public static final double rightHPAlignmentX = leftHPAlignmentX;
  public static final double rightHPAlignmentY = -1.38;
  public static final double rightHPThetaAlignment = Math.toRadians(-6);
  

  // private PIDController xController = new PIDController(kPX, 0, 0.001);
  // private PIDController yController = new PIDController(kPY, 0, 0.001);
  // private PIDController thetaController = new PIDController(kPTheta, 0, 0.001);
  private ProfiledPIDController xController = new ProfiledPIDController(kPX, 0, 0.001, new TrapezoidProfile.Constraints(3, 20));
  private ProfiledPIDController yController = new ProfiledPIDController(kPY, 0, 0.001, new TrapezoidProfile.Constraints(3, 20));
  private ProfiledPIDController thetaController = new ProfiledPIDController(kPTheta, 0, 0.001, new TrapezoidProfile.Constraints(2 * Math.PI, Math.PI * 20));

  private PIDController HPXController = new PIDController(kPX, 0, 0.001);
  private PIDController HPYController = new PIDController(kPY, 0, 0.001);
  private ProfiledPIDController HPThetaController = new ProfiledPIDController(kPTheta, 0, 0.001, new TrapezoidProfile.Constraints(2 * Math.PI, Math.PI * 20));

  private HolonomicDriveController holonomicController = new HolonomicDriveController(HPXController, HPYController, HPThetaController);
  private Pose2d nearestHP;

  private Pose2d leftCameraPose = new Pose2d();
  private Pose2d rightCameraPose = new Pose2d();

  public Pose2d cameraPose = new Pose2d();
  
  

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

    forwardLimiter = new SlewRateLimiter(5, -5, 0);
    strafeLimiter = new SlewRateLimiter(5, -5, 0);

    xController.setTolerance(kXTolerance);
    yController.setTolerance(kYTolerance);
    thetaController.setTolerance(kThetaTolerance);
    thetaController.enableContinuousInput(-Math.PI/2, Math.PI/2);
    HPThetaController.enableContinuousInput(-Math.PI, Math.PI);
    HPThetaController.setTolerance(Math.toRadians(2));
    holonomicController.setTolerance(new Pose2d(0.05, 0.05, new Rotation2d(0.05)));

    vision = new Vision(this);
  //   for(int i = 0; i < 10; i ++) {
  //     var poses = vision.getAllVisionEstimates(getPose());
  //     // var estimatePoseRight = vision.getEstimatedGlobalPoseRight(getPose());
  //     // var estimatePoseLeft = vision.getEstimatedGlobalPoseLeft(getPose());
  //     if (poses.size() > 0){
  //     poses.forEach(x -> {
  //       cameraPose = x.get().pose();
  //       swerve.addVisionMeasurement(cameraPose, Utils.fpgaToCurrentTime(x.get().timestampSeconds()), VecBuilder.fill(0.01,0.01,0.01));
  //     });
  //     Commands.waitSeconds(0.2).execute();
  //   }
  //   swerve.resetPose(cameraPose);
  // };

  }

  @Override
  public void periodic() {
    var poses = vision.getAllVisionEstimates(getPose());
    // var estimatePoseRight = vision.getEstimatedGlobalPoseRight(getPose());
    // var estimatePoseLeft = vision.getEstimatedGlobalPoseLeft(getPose());
    if (poses.size() > 0){
    poses.forEach(x -> {
      cameraPose = x.get().pose();
      swerve.addVisionMeasurement(cameraPose, Utils.fpgaToCurrentTime(x.get().timestampSeconds()), x.get().stdDevs());
    });
    }
    // if (estimatePoseRight.isPresent()){
    //   rightCameraPose = estimatePoseRight.get().pose();
    //   swerve.addVisionMeasurement(rightCameraPose, Utils.fpgaToCurrentTime(estimatePoseRight.get().timestampSeconds()), estimatePoseRight.get().stdDevs());
    // }
    // if (estimatePoseLeft.isPresent()){
    //   leftCameraPose = estimatePoseLeft.get().pose();
    //   swerve.addVisionMeasurement(leftCameraPose, Utils.fpgaToCurrentTime(estimatePoseLeft.get().timestampSeconds()), estimatePoseLeft.get().stdDevs());

    // }

    SmartDashboard.putNumber("Global Pose X", getPose().getX());
    SmartDashboard.putNumber("Global Pose Y", getPose().getY());
    SmartDashboard.putNumber("Global Pose Rotation", getPose().getRotation().getDegrees());
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
    
    public void driveRobotCentricInstant(ChassisSpeeds speeds){
      setTime = (new Date()).getTime();
      swerve.setControl(
        SwerveConfig.robotCentric
        .withVelocityX(speeds.vxMetersPerSecond)
        .withVelocityY(speeds.vyMetersPerSecond)
      .withRotationalRate(speeds.omegaRadiansPerSecond * 0.5)
    );
  }

  public Command driveRobotCentricInstantCommand(Supplier<ChassisSpeeds> speeds) {
    return run(() -> driveRobotCentricInstant(speeds.get()));
  }

  public void brake(){
    swerve.setControl(SwerveConfig.brake);
  }

  public Rotation2d getRotation2d(){
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

  public double getSpeed(){
    return Math.hypot(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond);
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


  public Command autoAlignCommand(boolean left, Supplier<Boolean> l4, Supplier<Boolean> l4Score, Supplier<Boolean> hpAlign) {
    return startRun(()->{
      // Set the target pose based on the alignment side
      double targetX;
      double targetY;
      double targetTheta;

      targetX = left ? leftHPAlignmentX : rightHPAlignmentX;
      targetY = left ? leftHPAlignmentY : rightHPAlignmentY;
      targetTheta = left ? leftHPThetaAlignment : rightHPThetaAlignment;
      
      xController.setGoal(targetX);
      yController.setGoal(targetY);
      thetaController.setGoal(targetTheta);
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
        SmartDashboard.putNumber("AutoAlign/xError", xController.getPositionError());
        SmartDashboard.putNumber("AutoAlign/yError", yController.getPositionError());
        SmartDashboard.putNumber("AutoAlign/xPos", visionPose.getX());
        SmartDashboard.putNumber("AutoAlign/yPos", visionPose.getY());
        SmartDashboard.putNumber("AutoAlign/xTarget", xController.getGoal().position);
        SmartDashboard.putNumber("AutoAlign/yTarget", yController.getGoal().position);
        SmartDashboard.putNumber("AutoAlign/thetaError", thetaController.getPositionError());
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
      xController.setGoal(targetX);
      yController.setGoal(targetY);
      thetaController.setGoal(targetTheta);
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
        SmartDashboard.putNumber("AutoAlign/xError", xController.getPositionError());
        SmartDashboard.putNumber("AutoAlign/yError", yController.getPositionError());
        SmartDashboard.putNumber("AutoAlign/thetaError", thetaController.getPositionError());
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
      xController.setGoal(targetX);
      yController.setGoal(targetY);
      thetaController.setGoal(targetTheta);

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
        SmartDashboard.putNumber("AutoAlign/xError", xController.getPositionError());
        SmartDashboard.putNumber("AutoAlign/yError", yController.getPositionError());
        SmartDashboard.putNumber("AutoAlign/thetaError", thetaController.getPositionError());
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
      xController.setGoal(targetX);
      xController.setTolerance(kXTolerance);
      yController.setGoal(targetY);
      yController.setTolerance(kYTolerance);
      thetaController.setGoal(targetTheta);
      thetaController.setTolerance(kThetaTolerance);
      xController.setGoal(targetX);
      yController.setGoal(targetY);
      thetaController.setGoal(targetTheta);

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
        SmartDashboard.putNumber("AutoAlign/xError", xController.getPositionError());
        SmartDashboard.putNumber("AutoAlign/yError", yController.getPositionError());
        SmartDashboard.putNumber("AutoAlign/thetaError", thetaController.getPositionError());
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


  public Command autoAlignHPCommand(boolean left) {
    return startRun(()->{
      // Set the target pose based on the alignment side
      double targetX = left ? leftHPAlignmentX : rightHPAlignmentX;
      double targetY = left ? leftHPAlignmentY : rightHPAlignmentY;
      double targetTheta = left ? leftHPThetaAlignment : rightHPThetaAlignment;

      // Set the PID controllers' setpoints
      xController.setGoal(targetX);
      xController.setTolerance(kHPXTolerance);
      yController.setGoal(targetY);
      yController.setTolerance(kHPYTolerance);
      thetaController.setGoal(targetTheta);
      thetaController.setTolerance(kHPThetaTolerance);

    },() -> {
      // Get the latest vision measurement (robot pose in tag space)
      Optional<Pose2d> visionOpt = vision.getRobotInTagSpace(left, true);
     
      if (visionOpt.isPresent()) {
        Pose2d visionPose = visionOpt.get();
       
       
        // Calculate corrections using your PID controllers
        double xPower = xController.calculate(visionPose.getX());
        double yPower = yController.calculate(visionPose.getY());
        double thetaPower = thetaController.calculate(visionPose.getRotation().getRadians());
       
        // Publish debug values to SmartDashboard (optional)
        SmartDashboard.putNumber("AutoAlign/xError", xController.getPositionError());
        SmartDashboard.putNumber("AutoAlign/yError", yController.getPositionError());
        SmartDashboard.putNumber("AutoAlign/thetaError", thetaController.getPositionError());
        SmartDashboard.putNumber("AutoAlign/xPower", xPower);
        SmartDashboard.putNumber("AutoAlign/yPower", yPower);
        SmartDashboard.putNumber("AutoAlign/thetaPower", thetaPower);
       
        // Command the drive: Here we send the PID outputs as chassis speeds
        driveRobotCentric(new ChassisSpeeds(
            Math.max(-2.5, Math.min(2.5, -xPower)),
            Math.max(-2.5, Math.min(2.5, -yPower)),
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

  public Pose2d getNearestHP(){
    Pose2d[] hpPoses = new Pose2d[]{FieldPositions.HP1, FieldPositions.HP2, FieldPositions.HP12, FieldPositions.HP13};
    Pose2d currentPose = getPose();
    nearestHP = hpPoses[0];
    double minDistance = currentPose.getTranslation().getDistance(hpPoses[0].getTranslation());

    for (Pose2d hp : hpPoses) {
      double dist = currentPose.getTranslation().getDistance(hp.getTranslation());
      if (dist < minDistance) {
          nearestHP = hp;
          minDistance = dist;
      }
    }
    return nearestHP;
  }

  public Command autoAlignNearestHPCommand(){
    return startRun(() -> {
      nearestHP = getNearestHP();

  }, () -> {
    double distance = getPose().getTranslation().getDistance(nearestHP.getTranslation());

// Only start rotating robot when within a certain distance of the HP station
Rotation2d desiredHeading = 
    (distance > 1.3) ? getPose().getRotation() : nearestHP.getRotation(); // tune distance so it swoops the least but doesnt overshoot

driveRobotCentric(holonomicController.calculate(
    getPose(),
    nearestHP,
    0.0,
    desiredHeading
));
  }).until(() -> {
    // Terminate when at HP station inside the 0.05 tolerance
    return holonomicController.atReference();
});

}


 public Command PPHPAlign(){
  return Commands.defer(() -> {
  nearestHP = getNearestHP();

  // Create a list of waypoints from poses. Each pose represents one waypoint.
  // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
  // Figures out the angle of the straight line from the robot to the HP station
  Translation2d angle = nearestHP.getTranslation().minus(getPose().getTranslation());
  Rotation2d pathHeading = new Rotation2d(Math.atan2(angle.getY(), angle.getX()));
  List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      new Pose2d(getPose().getTranslation(), pathHeading),
      new Pose2d(nearestHP.getTranslation(), pathHeading)
  );

  PathConstraints constraints = Constants.SwerveConstants.pathConstraints; // The constraints for this path.
  //PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

  // Create the path using the waypoints created above
  PathPlannerPath path = new PathPlannerPath(
    waypoints,
    constraints,
    null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
    new GoalEndState(0.0, nearestHP.getRotation()) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.   
  );
  // Prevent the path from being flipped if the coordinates are already correct
  path.preventFlipping = true;
  driveRobotCentricCommand(() -> new ChassisSpeeds(0, 0, 0));
  return new SequentialCommandGroup(AutoBuilder.followPath(path));
  },
  Set.of(this));
  };
 }

