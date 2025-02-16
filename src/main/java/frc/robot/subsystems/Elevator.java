package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    public final SparkMax elevatorSparkMax1;
    public final SparkMax elevatorSparkMax2;
    private final RelativeEncoder elevatorEncoder;
    private final PIDController pidController;

    public Elevator(){
        //Initialize motors with correct ports
        elevatorSparkMax1 = new SparkMax(21, MotorType.kBrushless);
        elevatorSparkMax2 = new SparkMax(22, MotorType.kBrushless);
        
        elevatorEncoder = elevatorSparkMax1.getEncoder();

        //Tunes the PID gains- Adjust for better control and movement of elevator
        double kp = 1.0; //Proportional (Increase the number if moving too slow, decrease if oscillating)
        double ki = 0.0; //Integral (Stays at 0 unless there's a steady-state error)
        double kd = 0.01; //Derivate (Increase if overshoot target, decrease if sluggish/slow)

        
        //Initialize PID controller with motion constraints
        pidController = new PIDController(kp, ki, kd);
        pidController.setTolerance(0.5); //Acceptable error range
    }

    public void moveToPos (double targetPos) {
  
        elevatorSparkMax1.set(pidController.calculate(elevatorEncoder.getPosition(), targetPos));
    }

    public double getPos () {
        return elevatorEncoder.getPosition();
    }

    public void stop() {
        elevatorSparkMax1.stopMotor();
    }
}