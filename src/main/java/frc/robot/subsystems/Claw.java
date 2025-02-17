package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

// ngl idk what am doing so am stealing from test code

public class Claw extends SubsystemBase {
    
    private SparkMax moveMotor;
    private SparkMax rollerMotor;

    public Claw() {
        moveMotor = new SparkMax(24, MotorType.kBrushless);
        rollerMotor = new SparkMax(25, MotorType.kBrushless);
    }

    public void moveClaw(double speed) {
        moveMotor.set(speed);
    }

    public void changeRollerSpd(double speed) {
        rollerMotor.set(speed);
    }

    public void stopClaw() {
        moveMotor.stopMotor();
    }

    public void stopRoller() {
        rollerMotor.stopMotor();
    }

    public Command moveClawCommand(double speed) {
        return run(() -> moveClaw(speed));
    }

    public Command changeRollerSpdCommand(double speed) {
        return run(() -> changeRollerSpd(speed));
    }

    public Command stopClawCommand() {
        return run(this::stopClaw);
    }

    public Command stopRollerCommand() {
        return run(this::stopRoller);
    }

    @Override
    public void periodic() {
        // icl ts pmo sm u bfr rn
    }


}
