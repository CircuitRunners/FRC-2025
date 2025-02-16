package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Claw extends SubsystemBase {
    
    private SparkMax leftMotor;
    private SparkMax rightMotor;

    public Claw() {
        leftMotor = new SparkMax(24, MotorType.kBrushless);
        rightMotor = new SparkMax(25, MotorType.kBrushless);
    }

    public void grab() {
        leftMotor.set(1);
        rightMotor.set(-1);
    }

    public void release() {
        leftMotor.set(-1);
        rightMotor.set(1);
    }

    public Command grabCommand() {
        return run(() -> grab());
    }

    public Command releaseCommand() {
        return run(() -> release());
    }


}
