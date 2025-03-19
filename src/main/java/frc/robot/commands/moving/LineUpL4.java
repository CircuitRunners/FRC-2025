// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.moving;

import java.util.function.DoubleSupplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.swerve.Swerve;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LineUpL4 extends SequentialCommandGroup {
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
    DoubleSupplier dist1 = () -> drive.distSensor1.getDistance().getValueAsDouble();
    DoubleSupplier dist2 = () -> drive.distSensor2.getDistance().getValueAsDouble();
    // double error = ((dist1  + dist2) / 2 ) - SwerveConstants.distanceFromReef;
    addCommands(
        //drive.driveRobotCentricCommand(() ->new ChassisSpeeds(0, 0, Units.degreesToRadians(20) * Math.signum(dist1.getAsDouble()-dist2.getAsDouble()))).until(() -> Math.abs(dist1.getAsDouble()-dist2.getAsDouble())<SwerveConstants.distanceThreshold),
        // drive.driveRobotCentricCommand(() -> new ChassisSpeeds(0,0,Math.signum(dist1.getAsDouble() - dist2.getAsDouble()) * Units.degreesToRadians(50))).withTimeout((Math.atan((dist1.getAsDouble() + Units.inchesToMeters(21)))/dist2.getAsDouble()) / Math.toRadians(50)),
        // drive.driveToForwardDistanceCommand(((dist1.getAsDouble()  + dist2.getAsDouble()) / 2 ) - SwerveConstants.distanceFromReef, 
        //     0.5 * Math.signum((dist1.getAsDouble() + dist2.getAsDouble()) /2  - SwerveConstants.distanceFromReef)
        //                 )
        drive.driveToForwardDistanceCommand((dist1.getAsDouble()+dist2.getAsDouble())/2 - SwerveConstants.distanceFromReef,0.5 * Math.signum(dist1.getAsDouble() - SwerveConstants.distanceFromReef))
    );
    
  }
}