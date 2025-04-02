// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.moving.LineUpL4;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Drive;
import frc.robot.commands.scoring.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutonScoreL4 extends SequentialCommandGroup {
  /** Creates a new SocreL4. */
  public AutonScoreL4(Drive drive, Elevator elevator, Claw claw, boolean left) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive, elevator, claw);
    addCommands(
      drive.driveRobotCentricCommand(() -> new ChassisSpeeds(1, 0, 0)).withTimeout(0.6),
      new ScoreL4Auto(elevator, claw, drive),
      drive.driveRobotCentricCommand(() -> new ChassisSpeeds(-1, 0, 0)).withTimeout(0.305)
    );
  }
}