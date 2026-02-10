package frc.robot.commands.scoring;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.moving.MoveToL4;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;

public class ScoreL4AutoWithAlgaeRemoval extends SequentialCommandGroup {
  /** Creates a new ScoreL4. */
  public ScoreL4AutoWithAlgaeRemoval(Elevator elevator, Claw claw, Drive drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(elevator, claw, drive);
    addCommands(
      drive.driveRobotCentricCommand(() -> new ChassisSpeeds(-0.75, 0, 0)).withTimeout(0.5).andThen(drive.brakeCommand()),
      new MoveToL4(elevator, claw, drive),
      drive.driveRobotCentricCommand(() -> new ChassisSpeeds(0.75, 0, 0)).withTimeout(0.5-0.21).andThen(drive.brakeCommand()),
      claw.scoreL4(),
      elevator.moveToL3().withTimeout(0.3),
      drive.driveRobotCentricCommand(() -> new ChassisSpeeds(0.75, 0, 0)).withTimeout(0.3).andThen(drive.brakeCommand()),
      new Algae1(elevator, claw).withTimeout(0.5),
      claw.moveClawToHorizontalCommand().withTimeout(0.3),
      drive.driveRobotCentricCommand(() -> new ChassisSpeeds(-0.5, 0, 0 )).withTimeout(0.5),
      elevator.moveToBottom().withTimeout(0.3),
      drive.driveRobotCentricCommand(() -> new ChassisSpeeds())
      // drive.setLimitCommand(0.8)
      );
    //
  }
}