// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreL4 extends SequentialCommandGroup {
  /** Creates a new ScoreL4. */
  public ScoreL4(Elevator elevator, TestArm arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // addCommands(moveElevatorToL4(), moveArmL4(), runOuttakeOut());
    addRequirements(elevator, arm);
    addCommands(elevator.moveToL4(), arm.moveToL4(), arm.moveOuttakeCommand(0.5));
  }
}
