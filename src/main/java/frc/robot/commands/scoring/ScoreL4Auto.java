package frc.robot.commands.scoring;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.moving.MoveToL4;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;

public class ScoreL4Auto extends SequentialCommandGroup {
  /** Creates a new ScoreL4. */
  public ScoreL4Auto(Elevator elevator, Claw claw, Drive drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(elevator, claw, drive);
    addCommands(
      drive.driveRobotCentricCommand(() -> new ChassisSpeeds(-0.75, 0, 0)).withTimeout(0.69).andThen(drive.brakeCommand()),
      new MoveToL4(elevator, claw, drive),
      drive.driveRobotCentricCommand(() -> new ChassisSpeeds(0.75, 0, 0)).withTimeout(0.69).andThen(drive.brakeCommand()),
      claw.scoreL4(),
      drive.driveRobotCentricCommand(() -> new ChassisSpeeds(-0.75, 0, 0)).withTimeout(0.69).andThen(drive.brakeCommand()),
      elevator.moveToBottom(),
      drive.setLimitCommand(0.8)
      );
    //
  }
}