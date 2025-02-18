package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.Constants;

public class ScoreL3 extends SequentialCommandGroup {
    public ScoreL3(Elevator elevator, Claw claw){

        addRequirements(elevator,claw);
        addCommands(elevator.moveToL3(),claw.moveClawToPositionCommand(Constants.ClawConstants.l3EncoderValue),claw.runRollersOutCommand());
    }
}
