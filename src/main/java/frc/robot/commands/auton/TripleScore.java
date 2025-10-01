package frc.robot.commands.auton;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.moving.MoveToIntake;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;

public class TripleScore extends SequentialCommandGroup{
    public TripleScore(Drive drive, Elevator elevator, Claw claw, boolean left) {
        addRequirements(drive, claw, elevator);
        addCommands(
        new AutonScoreL43Coral(drive, elevator, claw, left),
        new ParallelCommandGroup(drive.PPHPAlign(), new MoveToIntake(elevator, claw, drive)),
        new SequentialCommandGroup(drive.driveRobotCentricCommand(() -> new ChassisSpeeds(0, 0, 0)).withTimeout(0.05).andThen(claw.autoIntakeCommand().until(() -> claw.isCoralInClaw()).withTimeout(1.5)).andThen(claw.runRollersInCommand().withTimeout(.2).finallyDo(claw::stopRoller))),
        new AutonScoreL43Coral(drive, elevator, claw, true),
        new ParallelCommandGroup(drive.PPHPAlign(), new MoveToIntake(elevator, claw, drive)),
        new SequentialCommandGroup(drive.driveRobotCentricCommand(() -> new ChassisSpeeds(0, 0, 0)).withTimeout(0.05).andThen(claw.autoIntakeCommand().until(() -> claw.isCoralInClaw()).withTimeout(1.5)).andThen(claw.runRollersInCommand().withTimeout(.2).finallyDo(claw::stopRoller))),
        new AutonScoreL43Coral(drive, elevator, claw, false),
        new MoveToIntake(elevator, claw, drive)
        );
    }
}
