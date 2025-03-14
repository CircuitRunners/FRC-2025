// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.moving;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LineUpL4 extends Command {
  /** Creates a new LineUpL4. */
  SwerveSubsystem swerve;
  boolean left;
  Command driveStrafeCommand;
  private boolean finished = false;
  public LineUpL4(SwerveSubsystem swerveSubsystem, boolean leftBranch) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
    this.swerve = swerveSubsystem;
    this.left = leftBranch;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(left) {
      driveStrafeCommand = swerve.driveToStrafeDistanceCommand(0.25,0.5);
    } else {
      driveStrafeCommand = swerve.driveToStrafeDistanceCommand(-0.25,0.5);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while (!finished) { 
      double dist1 = swerve.distSensor1.getDistance().getValueAsDouble();
      double dist2 = swerve.distSensor2.getDistance().getValueAsDouble();
  
      if(Math.abs(dist1-dist2)>SwerveConstants.distanceThreshold){
        swerve.drive(new ChassisSpeeds(0, 0, (dist1-dist2)*SwerveConstants.distanceCoeff));
      }
      else if((dist1 - SwerveConstants.distanceFromReef) > SwerveConstants.distanceThreshold){
        swerve.driveToForwardDistanceCommand(dist1-SwerveConstants.distanceFromReef, 0.5).execute();
      }
      else if((SwerveConstants.distanceFromReef - dist1) > SwerveConstants.distanceThreshold){ 
        swerve.driveToForwardDistanceCommand(dist1-SwerveConstants.distanceFromReef, 0.5).execute();
      }
      else {
        swerve.drive(new ChassisSpeeds());
        driveStrafeCommand.schedule();
        finished = true;
      }
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
