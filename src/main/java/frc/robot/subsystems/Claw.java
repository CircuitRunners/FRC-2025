package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;

// ngl idk what am doing so am stealing from test code

public class Claw extends SubsystemBase {
    
    private SparkMax moveMotor;
    private SparkMax roller1Motor;
    private SparkMax roller2Motor;

    private PIDController pidController;
    private AbsoluteEncoder clawEncoder;

    private double targetPos;

    public Claw() {
        moveMotor = new SparkMax(23, MotorType.kBrushless);
        roller1Motor = new SparkMax(24, MotorType.kBrushless);
        roller2Motor = new SparkMax(25, MotorType.kBrushless);

        double constP = 1; // proportional coefficient gain
        double constI = 1; // integral coefficient gain
        double constD = 1; // derivative coefficient gain

        pidController = new PIDController(constP, constI, constD);
        pidController.setTolerance(1);

        clawEncoder = moveMotor.getAbsoluteEncoder();
        targetPos = clawEncoder.getPosition(); // initialize targetPos so PID doesn't try calculating with a null value

    }

    public double getClawPos() {
        return clawEncoder.getPosition();
    }

    public double getTargetPos() {
        return targetPos;
    }

    public void setTargetPos(double desiredPos) {
        targetPos = desiredPos;
    }

    public void changeRollerSpd(double speed) {
        roller1Motor.set(speed);
        roller2Motor.set(-speed);
    }

    public void stopClaw() {
        setTargetPos(getClawPos()); // Hold claw position
    }

    public void stopRoller() {
        roller1Motor.stopMotor();
        roller2Motor.stopMotor();
    }


    // Commands

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
        // abcd efg hijk lmnop qrs tuv wx y and z
        moveMotor.setVoltage(pidController.calculate(getClawPos(),getTargetPos()));
    }

}
