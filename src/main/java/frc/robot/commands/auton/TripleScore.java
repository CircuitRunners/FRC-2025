package frc.robot.commands.auton;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.moving.MoveToIntake;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;

public class TripleScore extends SequentialCommandGroup{
    private Drive drive;
    private Claw claw;
    public TripleScore(Drive drive, Elevator elevator, Claw claw, boolean left) {
        this.drive = drive;
        this.claw = claw;
        addRequirements(drive, claw, elevator);
        addCommands(
        new AutonScoreL43Coral(drive, elevator, claw, left),
        new ParallelCommandGroup(drive.PPHPAlign(), new MoveToIntake(elevator, claw, drive)),
        new SequentialCommandGroup(drive.driveRobotCentricCommand(() -> new ChassisSpeeds(0, 0, 0)).withTimeout(0.05).andThen(claw.autoIntakeCommand().until(() -> claw.isCoralInClaw()).withTimeout(1.5)).andThen(Commands.either(claw.runRollersInCommand().withTimeout(.2), jiggle(drive).alongWith(claw.autoIntakeCommand()).repeatedly().until(claw::isCoralInClaw).andThen(claw.runRollersInCommand().withTimeout(.2)), claw::isCoralInClaw)).finallyDo(claw::stopRoller)),
        new AutonScoreL43Coral(drive, elevator, claw, true),
        new ParallelCommandGroup(drive.PPHPAlign(), new MoveToIntake(elevator, claw, drive)),
        new SequentialCommandGroup(drive.driveRobotCentricCommand(() -> new ChassisSpeeds(0, 0, 0)).withTimeout(0.05).andThen(claw.autoIntakeCommand().until(() -> claw.isCoralInClaw()).withTimeout(1.5)).andThen(Commands.either(claw.runRollersInCommand().withTimeout(.2), jiggle(drive).alongWith(claw.autoIntakeCommand()).repeatedly().until(claw::isCoralInClaw).andThen(claw.runRollersInCommand().withTimeout(.2)), claw::isCoralInClaw)).finallyDo(claw::stopRoller)),
        new AutonScoreL43Coral(drive, elevator, claw, false),
        new MoveToIntake(elevator, claw, drive)
        );
    }
    public static Command jiggle(Drive drive) {
        return new SequentialCommandGroup(
            drive.driveRobotCentricInstantCommand(() -> new ChassisSpeeds(2, 0, 0)).withTimeout(0.3),
            drive.driveRobotCentricInstantCommand(() -> new ChassisSpeeds(0, 0, 0)).withTimeout(0.1),
            drive.driveRobotCentricInstantCommand(() -> new ChassisSpeeds(-3, 0, 0)).withTimeout(0.7)
        );
    }

}
