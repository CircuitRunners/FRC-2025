// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.lib.swerve.SwerveConfig;
import frc.lib.utils.PathPlannerUtil;
import frc.robot.Constants.DriverConstants;
import frc.robot.commands.moving.*;
import frc.robot.commands.scoring.*;
import frc.robot.generated.TunerConstants;
import frc.robot.io.*;
import frc.robot.subsystems.*;

public class Robot extends TimedRobot {
  private Drive drive;
  private Elevator elevator;
  private Claw claw;
  private DriverControls driverControls;
  private ManipulatorControls manipulatorControls;
  private Command m_autonomousCommand;
  private SendableChooser<Command> autoChooser = new SendableChooser<>();


  @Override
  public void robotInit() {
    // DataLogManager.start("logs");
    configureSubsystems();
    configureAutos();
  }
  
  @Override
  public void driverStationConnected() {
    configureBindings();
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
    elevator.resetTargetPos().schedule();
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
    PathPlannerUtil.configure(drive, true);
    NamedCommands.registerCommand("do nothing", Commands.none());
    autoChooser = AutoBuilder.buildAutoChooser("do nothing");
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    //register named commands
    NamedCommands.registerCommand("MoveToIntake", new MoveToIntake(elevator, claw));
    NamedCommands.registerCommand("AutoIntake", claw.autoIntakeCommand());
    NamedCommands.registerCommand("ScoreL1", new ScoreL1(elevator, claw));
    NamedCommands.registerCommand("ScoreL2", new ScoreL2(elevator, claw));
    NamedCommands.registerCommand("ScoreL3", new ScoreL3(elevator, claw));
    NamedCommands.registerCommand("ScoreL4", new ScoreL4(elevator, claw));
  }

  private void configureBindings() {
    
    // ------------------------------- DRIVER CONTROLS ---------------------------------------------------------
    driverControls = new DriverControls(DriverConstants.driverPort);
    drive.setDefaultCommand(drive.driveFieldCentricCommand(() -> SwerveConfig.toChassisSpeeds(driverControls)));
    driverControls.increaseLimit().onTrue(drive.increaseLimitCommand());
    driverControls.decreaseLimit().onTrue(drive.decreaseLimitCommand());
    driverControls.start().onTrue(drive.resetGyroCommand());
    // driverControls.a().whileTrue(PathPlannerUtil.getAutoCommand("Mid Preload to L4"));

    // driverControls.y().onTrue(elevator.moveElevatorUp()).onFalse(elevator.stopCommand());

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
    manipulatorControls.moveClawL4().onTrue(claw.moveClawToL4Command());
    manipulatorControls.scoreL4().onTrue(claw.scoreL4());
    manipulatorControls.runRollersIn().onTrue(claw.runRollersInCommand()).onFalse(claw.stopRollersCommand());
    manipulatorControls.runRollersOut().onTrue(claw.runRollersOutCommand()).onFalse(claw.stopRollersCommand());
    manipulatorControls.b().onTrue(claw.runRollersOutSlowCommand()).onFalse(claw.stopRollersCommand());

    //overall controls
    manipulatorControls.moveElevatorBottom().onTrue(new MoveToIntake(elevator, claw));
    manipulatorControls.moveToL1().onTrue(new MoveToL1(elevator, claw));
    manipulatorControls.moveToL2().onTrue(new MoveToL2(elevator, claw));
    manipulatorControls.moveToL3().onTrue(new MoveToL3(elevator, claw));
    manipulatorControls.moveToL4().onTrue(new MoveToL4(elevator, claw));
    
  }

  private void configureSubsystems() {
    drive = new Drive(TunerConstants.createDrivetrain(), false);
    drive.resetGyroCommand().schedule();
    elevator = new Elevator();
    // double prevLimit = Drive.limit;
    // elevator.isDrivingPrecarious().whileTrue(drive.setLimitCommand(0.2)).onFalse(drive.setLimitCommand(prevLimit)); 
    claw = new Claw();
  }

}