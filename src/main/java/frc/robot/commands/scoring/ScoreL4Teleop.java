package frc.robot.commands.scoring;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinder;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.moving.MoveToL4;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;

public class ScoreL4Teleop extends SequentialCommandGroup {
  /** Creates a new ScoreL4. */
  public ScoreL4Teleop(Elevator elevator, Claw claw, Drive drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(elevator, claw, drive);
    addCommands(
      claw.scoreL4(),
      elevator.moveToBottom()
      );
    //
  }
}