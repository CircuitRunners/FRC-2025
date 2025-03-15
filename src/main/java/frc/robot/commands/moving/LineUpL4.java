// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.moving;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LineUpL4 extends Command {
  /** Creates a new LineUpL4. */
  Drive drive;
  boolean left;
  Command driveStrafeCommand;
  Transform3d camToAprilTag;
  private boolean finished = false;
  public LineUpL4(Drive Drive, boolean leftBranch) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Drive);
    this.drive = Drive;
    this.left = leftBranch;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(left) {
        drive.visionRunning = false;
        camToAprilTag = drive.vision.frontRightCam.getAllUnreadResults().get(drive.vision.frontRightCam.getAllUnreadResults().size()).getBestTarget().bestCameraToTarget;
        driveStrafeCommand = drive.driveToStrafeDistanceCommand(camToAprilTag.getY() - SwerveConstants.aprilTagToRightCam.getY(),0.5);
    } else {
        drive.visionRunning = false;
        camToAprilTag = drive.vision.frontLeftCam.getAllUnreadResults().get(drive.vision.frontLeftCam.getAllUnreadResults().size()).getBestTarget().bestCameraToTarget;
        driveStrafeCommand = drive.driveToStrafeDistanceCommand(camToAprilTag.getY() - SwerveConstants.aprilTagToLeftCam.getY(),0.5);
    }
    drive.visionRunning = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while (!finished) { 
      double dist1 = drive.distSensor1.getDistance().getValueAsDouble();
      double dist2 = drive.distSensor2.getDistance().getValueAsDouble();
  
      if(Math.abs(dist1-dist2)>SwerveConstants.distanceThreshold){
        drive.driveRobotCentric(new ChassisSpeeds(0, 0, (dist1-dist2)*SwerveConstants.distanceCoeff));
      }
      else if((dist1 - SwerveConstants.distanceFromReef) > SwerveConstants.distanceThreshold){
        drive.driveToForwardDistanceCommand(dist1-SwerveConstants.distanceFromReef, 0.5).execute();
      }
      else if((SwerveConstants.distanceFromReef - dist1) > SwerveConstants.distanceThreshold){ 
        drive.driveToForwardDistanceCommand(dist1-SwerveConstants.distanceFromReef, 0.5).execute();
      }
      else {
        drive.brake();
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