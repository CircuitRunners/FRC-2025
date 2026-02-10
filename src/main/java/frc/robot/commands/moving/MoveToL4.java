// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.moving;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveToL4 extends SequentialCommandGroup {
  /** Creates a new MoveToL4. */
  public MoveToL4(Elevator elevator, Claw claw, Drive drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(elevator, claw);
    addCommands(
      claw.runRollersInCommand().withDeadline(Commands.waitSeconds(0.02)),
      new ParallelCommandGroup(
      elevator.moveToL4(), 
      Commands.waitSeconds(0.2)
        .andThen(claw.runRollersInCommandAlt().withDeadline(Commands.waitSeconds(0.09)))
        .andThen(Commands.waitSeconds(0.02))
        .andThen(claw.moveClawToL4Command())).withTimeout(1.5)
    );
  }
}
