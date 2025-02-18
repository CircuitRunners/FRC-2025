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
        // Call the stop method from the Elevator subsystem
        elevator.stop();
    }

    public boolean isFinished () {
        return false;
    }

}
