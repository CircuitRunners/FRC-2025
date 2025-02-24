package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

    public final SparkMax elevatorSparkMax1;
    public final SparkMax elevatorSparkMax2;
    private final AbsoluteEncoder elevatorEncoder;
    private final PIDController pidController;
    private double targetPos;
    private String targetState;

    public Elevator(){
        //Initialize motors with correct ports
        elevatorSparkMax1 = new SparkMax(ElevatorConstants.elevatorPort1, MotorType.kBrushless);
        elevatorSparkMax2 = new SparkMax(ElevatorConstants.elevatorPort2, MotorType.kBrushless);
        
        SparkBaseConfig spark1Config = new SparkMaxConfig().idleMode(IdleMode.kBrake);
        SparkBaseConfig spark2Config = new SparkMaxConfig().follow(ElevatorConstants.elevatorPort1, true).idleMode(IdleMode.kBrake);

        elevatorSparkMax1.configure(spark1Config, null, null);
        elevatorSparkMax2.configure(spark2Config, null, null);

        elevatorEncoder = elevatorSparkMax1.getAbsoluteEncoder();

        //Tunes the PID gains- Adjust for better control and movement of elevator
        double kp = 0.05; //Proportional (Increase the number if moving too slow, decrease if oscillating)  NEEDS TO BE TUNED
        double ki = 0.0; //Integral (Stays at 0 unless there's a steady-state error)  NEEDS TO BE TUNED
        double kd = 0.0; //Derivate (Increase if overshoot target, decrease if sluggish/slow)  NEEDS TO BE TUNED

        
        //Initialize PID controller with motion constraints
        pidController = new PIDController(kp, ki, kd);
        pidController.setTolerance(0.5); //Acceptable error range
    }

    // Move the elevator to the specified position
    public void moveToPos(double targetPos) {
        this.targetPos = targetPos;
    }

    public double getElevatorPos () {
        return elevatorEncoder.getPosition();
    }    

    public double getTargetPos () {
        return targetPos;
    }    

    public Command stopCommand() {
        return run(this::stop);
    }

    public void stop() {
        SmartDashboard.putString("elevator state", "stopped");
        elevatorSparkMax1.setVoltage(0);       elevatorSparkMax1.stopMotor();
        elevatorSparkMax2.stopMotor();
    }

    public Command moveToL4() {
        targetState = "L4";
        // SmartDashboard.putString("elevator state", "moving to " + targetState);
        return run(() -> {
            targetState = "L4";
            moveToPos(ElevatorConstants.l4EncoderValue);
        });
    }

    public Command moveToL3() {
        targetState = "L3";
        // SmartDashboard.putString("elevator state", "moving to " + targetState);
        return run(() ->{
            targetState = "L3";
            moveToPos(ElevatorConstants.l3EncoderValue);
            });
    }

    public Command moveToL2() {
        targetState = "L2";
        // SmartDashboard.putString("elevator state", "moving to " + targetState);
        return run(() -> {
            targetState = "L2";
            moveToPos(ElevatorConstants.l2EncoderValue);
        });
    }

    public Command moveToL1() {
        targetState = "L1";
        // SmartDashboard.putString("elevator state", "moving to " + targetState);
        return run(() -> {
            targetState = "L1";
            moveToPos(ElevatorConstants.l1EncoderValue);
        });
    }
    
    public Command moveToBottom() {
        targetState = "bottom";
        // SmartDashboard.putString("elevator state", "moving to " + targetState);
        return run(() -> {
            targetState = "bottom";
            moveToPos(ElevatorConstants.minEncoderValue);
        });
    }

    public boolean isAtTarget(){
        return Math.abs(getElevatorPos() - getTargetPos()) < ElevatorConstants.tolerance;
    }

    public Command moveElevatorUp() {
        return run(() -> elevatorSparkMax1.set(-0.5));
    }

    public Command moveElevatorDown() {
        return run(() -> elevatorSparkMax1.set(0.5));
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("elevator position",getElevatorPos());
        SmartDashboard.putNumber("elevator target",getTargetPos());
        
        // if (isAtTarget()) {
        //     SmartDashboard.putString("elevator state", "at target ");
        //     SmartDashboard.putString("elevator target state", targetState);
        //     stop();
        // } else {
        //     SmartDashboard.putString("elevator state", "moving ");
        //     SmartDashboard.putString("elevator target state", targetState);
        //     var output = pidController.calculate(elevatorEncoder.getPosition(), targetPos);
        //     elevatorSparkMax1.setVoltage(output);
        // }
    }
}