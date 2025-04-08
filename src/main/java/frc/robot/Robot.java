// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
// import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.swerve.SwerveConfig;
import frc.lib.utils.PathPlannerUtil;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.moving.*;
import frc.robot.commands.scoring.*;
import frc.robot.generated.TunerConstants;
import frc.robot.io.*;
import frc.robot.subsystems.*;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import frc.robot.commands.auton.*;

public class Robot extends TimedRobot {
  private Drive drive;
  private Elevator elevator;
  private Claw claw;
  private DriverControls driverControls;
  private ManipulatorControls manipulatorControls;
  private Command m_autonomousCommand;
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  Thread m_visionThread;


  // public Robot() {
  //   configureAutos();
  // }

  @Override
  public void robotInit() {
    // DataLogManager.start("logs");
    configureSubsystems();
    configureAutos();

    DataLogManager.logNetworkTables(false);
    // CameraServer.startAutomaticCapture();


    /**
     * If this vision thread doesnt work, comment out lines 93-124 and uncomment line 87
     */
    // m_visionThread =
    //     new Thread(
    //         () -> {
    //           UsbCamera camera = CameraServer.startAutomaticCapture();
    //           camera.setResolution(640, 480);

    //           CvSink cvSink = CameraServer.getVideo();
    //           CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

    //           Mat mat = new Mat();

    //           while (!Thread.interrupted()) {
    //             if (cvSink.grabFrame(mat) == 0) {
    //               outputStream.notifyError(cvSink.getError());
    //               continue;
    //             }

    //             Core.rotate(mat, mat, Core.ROTATE_90_COUNTERCLOCKWISE);
    //             int barWidth = 4;
    //             Imgproc.rectangle(
    //               mat,
    //               new Point(mat.width() - barWidth / 2, 0),
    //               new Point(mat.width() + barWidth / 2, mat.height()),
    //               new Scalar(0, 255, 0),
    //               -1
    //             );
    //             outputStream.putFrame(mat);
    //           }
    //         });
    // m_visionThread.setDaemon(true);
    // m_visionThread.start();
  }
    
  
  
  @Override
  public void driverStationConnected() {
    configureBindings();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if(Robot.isSimulation()){
      drive.vision.simulationPeriodic(drive.getPose());
    }
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
    drive.resetOrientationAuto();
  }

  @Override
  public void teleopInit() {
    elevator.resetTargetPos().execute();;
    // drive.resetRotation(new Rotation2d(330));
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
    
    NamedCommands.registerCommand("MoveToIntake", new MoveToIntake(elevator, claw, drive));
    NamedCommands.registerCommand("AutoIntake", claw.autoIntakeCommand().until(() -> claw.isCoralInClaw()));
    NamedCommands.registerCommand("ScoreL1", new ScoreL1(elevator, claw,  drive));
    NamedCommands.registerCommand("ScoreL2", new ScoreL2(elevator, claw, drive));
    NamedCommands.registerCommand("ScoreL3", new ScoreL3(elevator, claw, drive));
    NamedCommands.registerCommand("ScoreL4Auto", new ScoreL4Auto(elevator, claw, drive));
    NamedCommands.registerCommand("AutonScoreL4", new AutonScoreL4(drive,elevator, claw, true));
    // NamedCommands.registerCommand("ScoreL4Auto", new ScoreL4Teleop(elevator, claw, drive));
    NamedCommands.registerCommand("do nothing", Commands.none());
    NamedCommands.registerCommand("brake", drive.brakeCommand());
    NamedCommands.registerCommand("Drive Robot Centric Forward", drive.driveRobotCentricCommand(() -> new ChassisSpeeds(1, 0, 0)));
    NamedCommands.registerCommand("Drive Robot Centric Forward", drive.driveRobotCentricCommand(() -> new ChassisSpeeds(-1, 0, 0)));

    PathPlannerUtil.configure(drive, true);

    // autoChooser = AutoBuilder.buildAutoChooser("taxi");
    autoChooser = new SendableChooser<Command>();
    autoChooser.addOption("long taxi", drive.driveRobotCentricCommand(() -> new ChassisSpeeds(0.6 * SwerveConstants.maxVelocityMPS, 0, 0)).withTimeout(7));
    autoChooser.setDefaultOption("scoreL4 auto no pathplanner",
      drive.driveRobotCentricCommand(() -> new ChassisSpeeds(0.75, 0, 0))
        .alongWith(claw.runRollersInCommand().withTimeout(2.8))
        .withTimeout(3)
        .andThen(new ScoreL4Auto(elevator, claw, drive)));
        // .andThen(drive.driveRobotCentricCommand(() -> new ChassisSpeeds(0, 0, Math.PI)).withTimeout(1).finallyDo(drive::brake)));
      autoChooser.addOption("scoreL4 auto remove algae no pathplanner",
      drive.driveRobotCentricCommand(() -> new ChassisSpeeds(0.75, 0, 0))
        .alongWith(claw.runRollersInCommand().withTimeout(2.8))
        .withTimeout(3)
        .andThen(new ScoreL4AutoWithAlgaeRemoval(elevator, claw, drive)));
        // .andThen(drive.driveRobotCentricCommand(() -> new ChassisSpeeds(0, 0, Math.PI)).withTimeout(1).finallyDo(drive::brake)));
    
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  private void configureBindings() {
    
    // ------------------------------- DRIVER CONTROLS ---------------------------------------------------------
    
    SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withDeadband(DriverConstants.stickDeadband);
    driverControls = new DriverControls(DriverConstants.driverPort);
    drive.setDefaultCommand(drive.driveFieldCentricCommand(() -> 
      driveRequest
        .withVelocityX(driverControls.driveForward())
        .withVelocityY(driverControls.driveStrafe())
        .withRotationalRate(driverControls.driveRotation() * 0.8)  
    ));
    // driverControls.increaseLimit().onTrue(drive.increaseLimitCommand());
    // driverControls.decreaseLimit().onTrue(drive.decreaseLimitCommand());
    // 

    Trigger rotateRight = new Trigger(() -> driverControls.rightBumper().getAsBoolean());
    Trigger rotateLeft = new Trigger(() -> (driverControls.leftBumper().getAsBoolean()));
    Trigger strafeRight = new Trigger(() -> (!driverControls.rightBumper().getAsBoolean() && driverControls.povRight().getAsBoolean()));
    Trigger strafeLeft = new Trigger(() -> (!driverControls.leftBumper().getAsBoolean() && driverControls.povLeft().getAsBoolean()));
    
    rotateRight.whileTrue(drive.driveRobotCentricCommand(() -> new ChassisSpeeds(0, 0, -0.5)));
    rotateLeft.whileTrue(drive.driveRobotCentricCommand(() -> new ChassisSpeeds(0, 0, 0.5)));
    double speed = 0.4;
    // driverControls.start().onTrue(Commands.runOnce(() -> drive.zeroGyro(0), drive));
    driverControls.start().onTrue(Commands.runOnce(() -> drive.seedFieldCentric()));
    strafeRight.whileTrue(drive.driveRobotCentricCommand(() -> new ChassisSpeeds(0, -speed, 0)));
    strafeLeft.whileTrue(drive.driveRobotCentricCommand(() -> new ChassisSpeeds(0, speed, 0)));
    
    driverControls.robotMoveForward().whileTrue(drive.driveRobotCentricCommand(() -> new ChassisSpeeds(speed, 0, 0)));
    driverControls.robotMoveBack().whileTrue(drive.driveRobotCentricCommand(() -> new ChassisSpeeds(-speed, 0, 0)));
    driverControls.leftTrigger().whileTrue(drive.driveRobotCentricCommand(() -> new ChassisSpeeds(0, 0, Math.toRadians(2))));
    driverControls.rightTrigger().whileTrue(drive.driveRobotCentricCommand(() -> new ChassisSpeeds(0, 0, Math.toRadians(2))));

    // driverControls.b().whileTrue(new ScoreL4Auto(elevator, claw, drive));

    driverControls.a().whileTrue(AutoBuilder.buildAuto("Left 2 Coral Preload L4"));
    driverControls.y().whileTrue(AutoBuilder.buildAuto("Right 2 Coral Preload L4"));

    driverControls.b().onTrue(drive.autoAlignCommand(true));

    // driverControls.back().onTrue(drive.toggleSysIdMode());
    
    // driverControls.y().whileTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // driverControls.a().whileTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // driverControls.b().whileTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // driverControls.x().whileTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    // ------------------------------- Manipulator Controls ---------------------------------------------------------
    manipulatorControls = new ManipulatorControls(DriverConstants.operatorPort);

    // // elevator controls
    // manipulatorControls.moveElevatorBottom().onTrue(elevator.moveToBottom());
    // manipulatorControls.moveElevatorL1().onTrue(elevator.moveToL1());
    // manipulatorControls.moveElevatorL2().onTrue(elevator.moveToL2());
    // manipulatorControls.moveElevatorL3().onTrue(elevator.moveToL3());
    // manipulatorControls.moveElevatorL4().onTrue(elevator.moveToL4());
    // // claw controls
    manipulatorControls.start().onTrue(claw.moveClawToHorizontalCommand());
    // manipulatorControls.moveClawL4().onTrue(claw.moveClawToL4Command());
    // manipulatorControls.scoreL4Algae2().onTrue(new ScoreL4Algae(elevator, claw));
    manipulatorControls.Algae1().onTrue(new Algae1(elevator, claw));
    
    manipulatorControls.Algae2().onTrue(new Algae2(elevator, claw));
    manipulatorControls.scoreL4().whileTrue(new ScoreL4(elevator, claw));
    manipulatorControls.x().onTrue(claw.runRollersInSlowCommand()).onFalse(claw.stopRollersCommand());
    // manipulatorControls.x().onTrue(claw.moveClawToHorizontalCommand());
    manipulatorControls.runRollersIn().onTrue(claw.runRollersInCommand()).onFalse(claw.stopRollersCommand());
    manipulatorControls.runRollersOut().onTrue(claw.runRollersOutAdjustCommand()).onFalse(claw.stopRollersCommand());
    // manipulatorControls.rightTrigger().onTrue(claw.runManualCommand(0.1)).onFalse(claw.stopClawCommand());
    // manipulatorControls.leftTrigger().onTrue(claw.runManualCommand(-0.1)).onFalse(claw.stopClawCommand());
    // manipulatorControls.start().whileTrue(elevator.moveElevatorDown()).onFalse(elevator.stopCommand().andThen(elevator.resetPos()));

    // manipulatorControls.start().onTrue(new ParallelCommandGroup(elevator.resetTargetPos(), claw.resetTargetPos()));
    // manipulatorControls.b().onTrue(claw.runRollersOutSlowCommand()).onFalse(claw.stopRollersCommand());

    //overall controls
    manipulatorControls.leftStick().onTrue(new MoveToIntake(elevator, claw, drive)).onFalse(Commands.runOnce(() -> {
      drive.setDefaultCommand(drive.driveFieldCentricCommand(() -> 
        driveRequest
          .withVelocityX(driverControls.driveForward())
          .withVelocityY(driverControls.driveStrafe())
          .withRotationalRate(driverControls.driveRotation() * 0.8)  
    ));
    }));

    // manipulatorControls.rightTrigger().whileTrue(elevator.moveElevatorUp()).onFalse(elevator.resetTargetPos());
    // manipulatorControls.leftTrigger().whileTrue(elevator.moveElevatorDown()).onFalse(elevator.resetTargetPos());

    manipulatorControls.back().onTrue(new MoveToIntake(elevator, claw, drive));
    manipulatorControls.moveToL1().onTrue(new MoveToL1(elevator, claw, drive));
    manipulatorControls.moveToL2().onTrue(new MoveToL2(elevator, claw, drive));
    manipulatorControls.moveToL3().onTrue(new MoveToL3(elevator, claw, drive));
    manipulatorControls.moveToL4().onTrue(new MoveToL4(elevator, claw, drive));
    manipulatorControls.leftTrigger().onTrue(claw.runManualCommand(-0.1)).onFalse(claw.zeroArm());
    manipulatorControls.rightTrigger().onTrue(claw.runManualCommand(0.2)).onFalse(claw.zeroArm());
    
    
  }

  private void configureSubsystems() {
    drive = new Drive(TunerConstants.createDrivetrain(), false);
    elevator = new Elevator();
    // elevator.isDrivingPrecarious().whileTrue(drive.setLimitCommand(0.2)).onFalse(drive.setLimitCommand(1)); 
    claw = new Claw();
  }

}
