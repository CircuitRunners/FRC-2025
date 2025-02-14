package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    public final SparkMax elevatorSparkMax1;
    public final SparkMax elevatorSparkMax2;

    private final RelativeEncoder elevatorEncoder;

    public Elevator(){
        elevatorSparkMax1 = new SparkMax(20, MotorType.kBrushless);
        elevatorSparkMax2 = new SparkMax(21, MotorType.kBrushless);
        elevatorEncoder = elevatorSparkMax1.getEncoder();
    }
}