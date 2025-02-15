package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class Elevate extends Command {
    
    private Elevator elevator = new Elevator();

    double targetPos;

    public Elevate (double targetPos) {
        this.targetPos = targetPos;
    }

    public void initialize () {}

    public void execute () {
        elevator.moveToPos(targetPos);
    }

    public void end (boolean interrupted) {
        elevator.elevatorSparkMax1.stopMotor();
        elevator.elevatorSparkMax2.stopMotor();
    }

    public boolean isFinished () {
        return false;
    }

}
