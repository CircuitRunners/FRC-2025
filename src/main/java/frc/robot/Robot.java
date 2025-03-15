// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
// import java.util.function.Supplier;

// import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
// import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.FieldCentric;
// import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
// import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.OperatorConstants;
// import frc.robot.Constants.SwerveConstants;
// import frc.robot.commands.moving.*;
import frc.robot.commands.scoring.*;
// import frc.robot.generated.TunerConstants;
import frc.robot.io.*;
import frc.robot.subsystems.*;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.CvSink;
// import edu.wpi.first.cscore.CvSource;
// import edu.wpi.first.cscore.UsbCamera;
// import edu.wpi.first.wpilibj.TimedRobot;
// import org.opencv.core.Mat;
// import org.opencv.core.Point;
// import org.opencv.core.Scalar;
// import org.opencv.imgproc.Imgproc;
// import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
// import java.io.File;
import swervelib.SwerveInputStream;

// import com.pathplanner.lib.path.PathConstraints;

public class Robot extends TimedRobot {
  // private Drive drive;
  private Elevator elevator;
  private Claw claw;
  private DriverControls driverControls;
  private ManipulatorControls manipulatorControls;
  private Command m_autonomousCommand;
  private SendableChooser<Command> autoChooser = new SendableChooser<>();


  
  private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));


  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverControls.driveForward(),
      () -> driverControls.driveStrafe())
    .withControllerRotationAxis(() -> driverControls.driveRotation())
    // .deadband(OperatorConstants.DEADBAND)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);
  
    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> driverControls.getRightX(),
                                                                                              () -> driverControls.getRightY())
                                                            .headingWhile(true);
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                              .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> driverControls.driveForward(),
                                                                        () -> driverControls.driveStrafe())
                                                                    .withControllerRotationAxis(() -> driverControls.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                              .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverControls.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                                2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverControls.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                                2))
                                                                                .headingWhile(true)
                                                                                .translationHeadingOffset(true)
                                                                                .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                    0));



  @Override
  public void robotInit() {
    // DataLogManager.start("logs");
    configureSubsystems();
    configureAutos();

  }
  
  @Override
  public void driverStationConnected() {
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = autoChooser.getSelected();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    
  }

  @Override
  public void autonomousPeriodic() {
  
  }

  @Override
  public void autonomousExit() {
  
  }

  @Override
  public void teleopInit() {
    elevator.resetTargetPos().execute();;
    // drive.resetGyroCommand().execute();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

  }
  
  @Override
  public void teleopExit() {
  
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  
  }

  @Override
  public void testExit() {
  }

  private void configureAutos() {
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    // NamedCommands.registerCommand("MoveToIntake", new MoveToIntake(elevator, claw, drive));
    NamedCommands.registerCommand("AutoIntake", claw.autoIntakeCommand());
    // NamedCommands.registerCommand("ScoreL1", new ScoreL1(elevator, claw,  drive));
    // NamedCommands.registerCommand("ScoreL2", new ScoreL2(elevator, claw, drive));
    // NamedCommands.registerCommand("ScoreL3", new ScoreL3(elevator, claw, drive));
    // NamedCommands.registerCommand("ScoreL4", new ScoreL4Algae(elevator, claw));
    // NamedCommands.registerCommand("ScoreL4Auto", new ScoreL4Auto(elevator, claw, drive));
    NamedCommands.registerCommand("do nothing", Commands.none());
    // NamedCommands.registerCommand("brake", drive.brakeCommand());
    // PathPlannerUtil.configure(drive, true);

    autoChooser = AutoBuilder.buildAutoChooser("taxi");
    // autoChooser.addOption("long taxi", drive.driveRobotCentricCommand(() -> new ChassisSpeeds(0.6 * SwerveConstants.maxVelocityMPS, 0, 0)).withTimeout(7));
    // autoChooser.setDefaultOption("scoreL4 auto no pathplanner", drive.driveRobotCentricCommand(() -> new ChassisSpeeds(0.75, 0, 0)).withTimeout(3).andThen(new ScoreL4Auto(elevator, claw, drive)));
    
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  private void configureBindings() {
    
    // ------------------------------- DRIVER CONTROLS ---------------------------------------------------------
    driverControls = new DriverControls(DriverConstants.driverPort);
    configureDriverBindings();
    // ------------------------------- Manipulator Controls ---------------------------------------------------------
    manipulatorControls = new ManipulatorControls(DriverConstants.operatorPort);

    // // elevator controls
    // manipulatorControls.moveElevatorBottom().onTrue(elevator.moveToBottom());
    // manipulatorControls.moveElevatorL1().onTrue(elevator.moveToL1());
    // manipulatorControls.moveElevatorL2().onTrue(elevator.moveToL2());
    // manipulatorControls.moveElevatorL3().onTrue(elevator.moveToL3());
    // manipulatorControls.moveElevatorL4().onTrue(elevator.moveToL4());
    // // claw controls
    manipulatorControls.moveClawHorizontal().onTrue(claw.moveClawToHorizontalCommand());
    // manipulatorControls.moveClawL4().onTrue(claw.moveClawToL4Command());
    manipulatorControls.scoreL4Algae2().onTrue(new ScoreL4Algae(elevator, claw));
    // manipulatorControls.Algae1().onTrue(new Algae1(elevator, claw, drive));
    // manipulatorControls.scoreL4().whileTrue(new ScoreL4Auto(elevator, claw, drive));
    manipulatorControls.runRollersIn().onTrue(claw.runRollersInCommand()).onFalse(claw.stopRollersCommand());
    manipulatorControls.runRollersOut().onTrue(claw.runRollersOutCommand()).onFalse(claw.stopRollersCommand());
    manipulatorControls.rightTrigger().onTrue(claw.runManualCommand(0.1)).onFalse(claw.stopClawCommand());
    manipulatorControls.leftTrigger().onTrue(claw.runManualCommand(-0.1)).onFalse(claw.stopClawCommand());
    manipulatorControls.start().whileTrue(elevator.moveElevatorDown()).onFalse(elevator.stopCommand().andThen(elevator.resetPos()));

    // manipulatorControls.start().onTrue(new ParallelCommandGroup(elevator.resetTargetPos(), claw.resetTargetPos()));
    // manipulatorControls.b().onTrue(claw.runRollersOutSlowCommand()).onFalse(claw.stopRollersCommand());

    //overall controls
    // manipulatorControls.moveElevatorBottom().onTrue(new MoveToIntake(elevator, claw, drive));
    // manipulatorControls.moveToL1().onTrue(new MoveToL1(elevator, claw, drive));
    // manipulatorControls.moveToL2().onTrue(new MoveToL2(elevator, claw, drive));
    // manipulatorControls.moveToL3().onTrue(new MoveToL3(elevator, claw, drive));
    // manipulatorControls.moveToL4().onTrue(new MoveToL4(elevator, claw, drive));
    
  }

  private void configureDriverBindings() {
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    // if (RobotBase.isSimulation())
    // {
    //   drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    // } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    }

    // if (Robot.isSimulation())
    // {
      // Pose2d target = new Pose2d(new Translation2d(1, 4),
      //                             Rotation2d.fromDegrees(90));
      // //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      // driveDirectAngleKeyboard.driveToPose(() -> target,
      //                                       new ProfiledPIDController(5,
      //                                                                 0,
      //                                                                 0,
      //                                                                 new Constraints(5, 2)),
      //                                       new ProfiledPIDController(5,
      //                                                                 0,
      //                                                                 0,
      //                                                                 new Constraints(Units.degreesToRadians(360),
      //                                                                                 Units.degreesToRadians(180))
                                            // ));
      driverControls.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverControls.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      // driverControls.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
      //                                               () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));


      driveAngularVelocity.driveToPose(drivebase::getClosestReefPose, 
                            new ProfiledPIDController(5,
                                                      0,
                                                      0,
                                                      new Constraints(5, 2)),
                            new ProfiledPIDController(5,
                                                      0,
                                                      0,
                                                      new Constraints(Units.degreesToRadians(360),
                                                                      Units.degreesToRadians(180))));

    driverControls.b().whileTrue(Commands.runEnd(() -> driveAngularVelocity.driveToPoseEnabled(true), () -> driveAngularVelocity.driveToPoseEnabled(false)));

    // }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity); // Overrides drive command above!

      driverControls.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverControls.y().whileTrue(drivebase.driveToForwardDistanceCommand(1.0, 0.2));
      driverControls.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverControls.back().whileTrue(drivebase.centerModulesCommand());
      driverControls.leftBumper().onTrue(Commands.none());
      driverControls.rightBumper().onTrue(Commands.none());
    } else
    {
      driverControls.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverControls.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverControls.start().whileTrue(Commands.none());
      driverControls.back().whileTrue(Commands.none());
      driverControls.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverControls.rightBumper().onTrue(Commands.none());
    }
  }
  
  double getDistance(Pose2d pose, Pose2d otherPose) {
    return Math.sqrt(Math.pow(pose.getX() - otherPose.getX(), 2) + Math.pow(pose.getY() - otherPose.getY(), 2));
  }



  private void configureSubsystems() {
    // drive = new Drive(TunerConstants.createDrivetrain(), false);
    // drive.zeroGyro();
    elevator = new Elevator();
    // elevator.isDrivingPrecarious().whileTrue(drive.setLimitCommand(0.2)).onFalse(drive.setLimitCommand(1)); 
    claw = new Claw();
  }

}
