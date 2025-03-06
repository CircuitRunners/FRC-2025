// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import java.util.Date;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.moving.MoveToL4;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreL4Auto extends SequentialCommandGroup {
  /** Creates a new ScoreL4. */
  public ScoreL4Auto(Elevator elevator, Claw claw, Drive drive) {
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(elevator, claw, drive);
    addCommands(
      drive.driveRobotCentricCommand(() -> new ChassisSpeeds(-0.5, 0, 0)).withTimeout(0.71).andThen(drive.brakeCommand()),
      new MoveToL4(elevator, claw, drive),
      drive.driveRobotCentricCommand(() -> new ChassisSpeeds(0.5, 0, 0)).withTimeout(0.71).andThen(drive.brakeCommand()),
      claw.scoreL4(),
      drive.driveRobotCentricCommand(() -> new ChassisSpeeds(-0.5, 0, 0)).withTimeout(0.71).andThen(drive.brakeCommand()),
      elevator.moveToL1(),
      drive.driveFieldCentricCommand(() -> new ChassisSpeeds())
      );
  }
}
