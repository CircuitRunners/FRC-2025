// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.moving.MoveToL4;
import frc.robot.subsystems.*;
import frc.robot.commands.scoring.*;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DoubleScore extends SequentialCommandGroup {
  /** Creates a new SocreL4. */
  public DoubleScore(Drive drive, Elevator elevator, Claw claw, boolean left) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive, claw);
    addCommands(
      drive.autoAlignCommand(left, () -> false, () -> false, () -> true),
      claw.autoIntakeCommand().until(() -> claw.isCoralInClaw()).withTimeout(3).andThen(claw.runRollersInCommand().withTimeout(0.3)),
      new ParallelCommandGroup(
        drive.autoAlignCommand(left, () -> false, () -> true).withTimeout(3),
        new MoveToL4(elevator, claw, drive)
      ),
      // new MoveToL4(elevator, claw, drive),
      // drive.autoAlignCommand(left, () -> false, () -> true).withTimeout(3),
      // drive.autoAlignCommand(left, () -> false, () -> true).withTimeout(3),
      // drive.autoAlignCommand(left, () -> false, () -> true).withTimeout(3),
      // drive.autoAlignCommand(left, () -> false, () -> true).withTimeout(3),
      // drive.autoAlignCommand(left, () -> false, () -> true).withTimeout(3),
      // drive.autoAlignCommand(left, () -> false, () -> true).withTimeout(3),
      claw.scoreL4(),
      new ParallelCommandGroup(
        elevator.moveToBottom(),
        drive.driveRobotCentricCommand(() -> new ChassisSpeeds(-2, 0.25, 0)).withTimeout(0.1).finallyDo(() -> drive.driveRobotCentric(new ChassisSpeeds(0, 0, 0)))
        ).withTimeout(0.3),
      drive.autoAlignCommand(left, () -> false, () -> false, () -> true),
      claw.autoIntakeCommand().until(() -> claw.isCoralInClaw()).withTimeout(3).andThen(claw.runRollersInCommand().withTimeout(0.3)),
      new ParallelCommandGroup(
        drive.autoAlignCommand(!left, () -> false, () -> true).withTimeout(3),
        new MoveToL4(elevator, claw, drive)
      ),
      // new MoveToL4(elevator, claw, drive),
      // drive.autoAlignCommand(!left, () -> false, () -> true).withTimeout(3),
      // drive.autoAlignCommand(!left, () -> false, () -> true).withTimeout(3),
      // drive.autoAlignCommand(!left, () -> false, () -> true).withTimeout(3),
      // drive.autoAlignCommand(!left, () -> false, () -> true).withTimeout(3),
      // drive.autoAlignCommand(!left, () -> false, () -> true).withTimeout(3),
      // drive.autoAlignCommand(!left, () -> false, () -> true).withTimeout(3),
      claw.scoreL4(),
      elevator.moveToBottom()
    );
  }
}
