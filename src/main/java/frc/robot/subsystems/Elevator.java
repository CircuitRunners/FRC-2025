package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;

public class Elevator 
extends SubsystemBase {

    public final SparkMax elevatorSparkMax1;
    public final SparkMax elevatorSparkMax2;
    private final RelativeEncoder elevatorEncoder;
    private final PIDController pidController;
    private double targetPos;
    private String targetState;
    private boolean running;

    public Elevator(){
        //Initialize motors with correct ports
        elevatorSparkMax1 = new SparkMax(ElevatorConstants.elevatorPort1, MotorType.kBrushless);
        elevatorSparkMax2 = new SparkMax(ElevatorConstants.elevatorPort2, MotorType.kBrushless);
        
        SparkBaseConfig spark1Config = new SparkMaxConfig().idleMode(IdleMode.kCoast);
        SparkBaseConfig spark2Config = new SparkMaxConfig().follow(ElevatorConstants.elevatorPort1, true).idleMode(IdleMode.kCoast);

        elevatorSparkMax1.configure(spark1Config, null, null);
        elevatorSparkMax2.configure(spark2Config, null, null);

        elevatorEncoder = elevatorSparkMax1.getEncoder();
        targetPos = getElevatorPos();

        //Tunes the PID gains- Adjust for better control and movement of elevator
        double kp = 0.045; //Proportional (Increase the number if moving too slow, decrease if oscillating)  NEEDS TO BE TUNED
        double ki = 0.0; //Integral (Stays at 0 unless there's a steady-state error)  NEEDS TO BE TUNED
        double kd = 0; //Derivate (Increase if overshoot target, decrease if sluggish/slow)  NEEDS TO BE TUNED
        
        //Initialize PID controller with motion constraints
        pidController = new PIDController(kp, ki, kd);
        pidController.setTolerance(1); //Acceptable error range
    }

    // Move the elevator to the specified position
    public void moveToPos(double targetPos) {
        running = true;
        this.targetPos = targetPos;
        pidController.setSetpoint(targetPos);
    }

    public Command resetTargetPos() {
        return runOnce(() -> moveToPos(getElevatorPos()));
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
        elevatorSparkMax1.setVoltage(0);       
        elevatorSparkMax1.stopMotor();
        elevatorSparkMax2.stopMotor();
    }

    public boolean isAtTarget(){
        return pidController.atSetpoint();
    }

    public Command moveElevatorCommand(double targetPosition, String stateName) {
        return new MoveElevatorCommand(this, targetPosition, stateName);
    }
    
    public Command moveToL4() {
        return moveElevatorCommand(ElevatorConstants.l4EncoderValue, "L4");
    }

    public Command pidTest() {
        targetState = "PID";
        // SmartDashboard.putString("elevator state", "moving to " + targetState);
        return run(() -> {
            targetState = "PID";
            moveToPos(-80);
        });
    }

    public Command moveToL3() {
        return moveElevatorCommand(ElevatorConstants.l3EncoderValue, "L3");
    }

    public Command moveToL2() {
        return moveElevatorCommand(ElevatorConstants.l2EncoderValue, "L2");
    }

    public Command moveToL1() {
        return moveElevatorCommand(ElevatorConstants.l1EncoderValue, "L1");
    }
    
    public Command moveToBottom() {
        return moveElevatorCommand(ElevatorConstants.minEncoderValue, "bottom");
    }

    

    public Command moveElevatorUp() {
        return run(() -> {
            SmartDashboard.putString("state", "moving");
            elevatorSparkMax1.set(-0.5);});
    }

    public Command moveElevatorDown() {
        return run(() -> {
            SmartDashboard.putString("state", "moving");
            elevatorSparkMax1.set(0.5);});
    }

    public boolean isElevatorAtBottom() {
        return (getElevatorPos() - ElevatorConstants.minEncoderValue < 5);
    }

    public Trigger isDrivingPrecarious() {
        return new Trigger(() -> !isElevatorAtBottom() || elevatorSparkMax1.get() > 0.5);
    }

    @Override
    public void periodic(){
        var output = pidController.calculate(getElevatorPos(), this.targetPos);
        elevatorSparkMax1.set(output);

        SmartDashboard.putNumber("elevator position",getElevatorPos());
        SmartDashboard.putNumber("elevator target",getTargetPos());
        SmartDashboard.putNumber("elevator target setpoint",pidController.getSetpoint());
        SmartDashboard.putBoolean("isAtTarget", isAtTarget());
        SmartDashboard.putString("elevator state", "moving ");
        // SmartDashboard.putString("elevator target state", targetState);
        SmartDashboard.putNumber("motor current", elevatorSparkMax1.getOutputCurrent());
        SmartDashboard.putNumber("output", output);
        SmartDashboard.putBoolean("running", running);
        SmartDashboard.updateValues();
    }

    public static class MoveElevatorCommand extends Command {
        private final Elevator elevator;
        private final double targetPosition;
        private final String targetState;

        public MoveElevatorCommand(Elevator elevator, double targetPosition, String targetState) {
            this.elevator = elevator;
            this.targetPosition = targetPosition;
            this.targetState = targetState;
            addRequirements(elevator);
        }

        @Override
        public void initialize() {
            elevator.targetState = targetState;
            elevator.moveToPos(targetPosition);
            SmartDashboard.putString("Elevator command", "Moving to " + targetState);
        }

        @Override
        public boolean isFinished() {
            return elevator.isAtTarget();
        }
    }
}
