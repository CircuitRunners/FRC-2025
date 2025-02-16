package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    public final SparkMax elevatorSparkMax1;
    public final SparkMax elevatorSparkMax2;
    private final RelativeEncoder elevatorEncoder;

    private final ProfiledPIDController elevatorPidController;

    public Elevator(){
        //Initialize the two motors
        elevatorSparkMax1 = new SparkMax(20, MotorType.kBrushless);
        elevatorSparkMax2 = new SparkMax(21, MotorType.kBrushless);
        
        elevatorEncoder = elevatorSparkMax1.getEncoder();

        //Tunes the PID gains -Adjusted for better control and movement of elevator
        double kp = 1.0; //Proportional (Increase the number if moving too slow, decrease if oscillating)
        double ki = 0.0; //Integral (Stays at 0 unless there's a steady-state error)
        double kd = 0.01; //Derivate (Increase if overshoot target, decrease if sluggish/slow)

        //Motion constraints
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(2000, 1500);
        
        //Initialize PID controller with motion constraints
        elevatorPidController = new ProfiledPIDController(kp, ki, kd, constraints);
        elevatorPidController.setTolerance(0.5); //Acceptable error range
    }

    public void moveToPos (double targetPos) {  
        elevatorSparkMax1.set(elevatorPidController.calculate(elevatorEncoder.getPosition(), targetPos));
    }

    public double getPos () {
        return elevatorEncoder.getPosition();
    }
}