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

    private static final double HOLD_POWER = 0.05;

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

    // Move the elevator to the specified position
    public void moveToPos(double targetPos) {
        // Get the PID output based on current position and target
        double output = pidController.calculate(elevatorEncoder.getPosition(), targetPos);
        // Set both motors to the same output to ensure synchronized movement
        elevatorSparkMax1.set(output);
        elevatorSparkMax2.set(output);
    }

    public double getPos () {
        return elevatorEncoder.getPosition();
    }

    public void holdPosition() {
        // Calculate the PID output for the current position
        double output = pidController.calculate(elevatorEncoder.getPosition(), elevatorEncoder.getPosition());
    
        // Ensure that the output isn't too low, in which case we use HOLD_POWER
        if (Math.abs(output) < HOLD_POWER) {
            output = (output > 0) ? HOLD_POWER : -HOLD_POWER; // Apply HOLD_POWER with correct sign
        }
    
        // Set both motors to the calculated output
        elevatorSparkMax1.set(output);
        elevatorSparkMax2.set(output);
    }
    

    public void stop() {
        elevatorSparkMax1.stopMotor();
        elevatorSparkMax2.stopMotor();
    }
}