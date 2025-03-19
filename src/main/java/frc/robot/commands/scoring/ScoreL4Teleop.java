package frc.robot.commands.scoring;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinder;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.moving.LineUpL4;
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
      new LineUpL4(drive, false),
      new MoveToL4(elevator, claw, drive),
      drive.driveRobotCentricCommand(() -> new ChassisSpeeds(0.5, 0, 0)).withTimeout(0.5).andThen(drive.brakeCommand()),
      // claw.scoreL4(),
      drive.driveToForwardDistanceCommand(-0.25, -0.5),
      elevator.moveToBottom()
      );
    //
  }
}