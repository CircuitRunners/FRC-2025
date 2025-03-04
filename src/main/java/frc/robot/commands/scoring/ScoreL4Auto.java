// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.moving.MoveToL4;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreL4Auto extends SequentialCommandGroup {
  /** Creates a new ScoreL4. */
  public ScoreL4Auto(Elevator elevator, Claw claw, Drive drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(elevator, claw, drive);
    addCommands(
      drive.driveRobotCentricCommand(() -> new ChassisSpeeds(0.25, 0, 0)).until(() -> drive.getChassisSpeeds().vxMetersPerSecond <= 0.05),
      claw.scoreL4(),
      drive.driveRobotCentricCommand(() -> new ChassisSpeeds(-0.255, 0, 0)).withTimeout(0.2),
      elevator.moveToBottom()
      );
    //
  }
}
