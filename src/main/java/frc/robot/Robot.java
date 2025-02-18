// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.swerve.SwerveConfig;
import frc.lib.utils.PathPlannerUtil;
import frc.robot.Constants.DriverConstants;
import frc.robot.commands.ScoreL4;
import frc.robot.generated.TunerConstants;
import frc.robot.io.DriverControls;
import frc.robot.subsystems.*;

public class Robot extends TimedRobot {
  private Drive drive;
  private Elevator elevator;
  private TestArm testArm;
  private DriverControls driverControls;
  private Command m_autonomousCommand;
  private final SendableChooser<Supplier<Command>> autoChooser = new SendableChooser<>();
  // private Vision vision;


  @Override
  public void robotInit() {
    // DataLogManager.start("logs");
    configureSubsystems();
    configureAutos();
    // vision = new Vision(drive::addVisionMeasurement);
  }
  
  @Override
  public void driverStationConnected() {
    configureBindings();
    // addPeriodic(vision::run, 0.01);
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
    m_autonomousCommand = autoChooser.getSelected().get();
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
    PathPlannerUtil.configure(drive);
    autoChooser.setDefaultOption("do nothing", () -> Commands.none());
    PathPlannerUtil.getAutos().forEach(autoName -> {
      autoChooser.addOption(autoName, () -> PathPlannerUtil.getAutoCommand(autoName));
    });
    SmartDashboard.putData("Auto Chooser", autoChooser);

    NamedCommands.registerCommand("ScoreL4", new ScoreL4(elevator, testArm));
  }

  private void configureBindings() {
    
    // ------------------------------- DRIVER CONTROLS ---------------------------------------------------------
    driverControls = new DriverControls(DriverConstants.driverPort);
    drive.setDefaultCommand(drive.driveFieldCentricCommand(() -> SwerveConfig.toChassisSpeeds(driverControls)));
    driverControls.increaseLimit().onTrue(drive.increaseLimitCommand());
    driverControls.decreaseLimit().onTrue(drive.decreaseLimitCommand());
    driverControls.start().onTrue(drive.resetGyroCommand());
  }

  private void configureSubsystems() {
    drive = new Drive(TunerConstants.createDrivetrain());
    elevator = new Elevator();
    testArm = new TestArm();
  }

}