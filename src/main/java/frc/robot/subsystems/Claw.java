
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public class Claw extends SubsystemBase {
    
    private SparkMax moveMotor;
    private SparkMax roller1Motor;
    private SparkMax roller2Motor;
    private CANrange canRangeSensor;

    private PIDController pidController;
    private AbsoluteEncoder clawEncoder;

    private double targetPos;

    private String targetState;

    public Claw() {
        moveMotor = new SparkMax(ClawConstants.moveMotorPort, MotorType.kBrushless);
        roller1Motor = new SparkMax(ClawConstants.rollerMotorPort1, MotorType.kBrushless);
        roller2Motor = new SparkMax(ClawConstants.rollerMotorPort2, MotorType.kBrushless);

        SparkBaseConfig r1mConfig = new SparkMaxConfig().idleMode(IdleMode.kCoast);
        SparkBaseConfig r2mConfig = new SparkMaxConfig().follow(ClawConstants.rollerMotorPort1,true).idleMode(IdleMode.kCoast);
        roller1Motor.configure(r1mConfig, null, null);
        roller2Motor.configure(r2mConfig, null, null);

        canRangeSensor = new CANrange(ClawConstants.canRangePort);

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
    }

    public boolean isCoralInClaw() {
        return canRangeSensor.getDistance().getValueAsDouble() < ClawConstants.coralSensorRange;
    }

    public void stopClaw() {
        //setTargetPos(getClawPos()); // Hold claw position
        moveMotor.stopMotor();
    }

    public void stopRoller() {
        roller1Motor.stopMotor();
        roller2Motor.stopMotor();
    }


    // Commands

    public Command changeRollerSpdCommand(double speed) {
        return run(() -> changeRollerSpd(speed));
    }
    
        public Command runRollersInCommand() {
            SmartDashboard.putString("rollers state", "running in");
            return changeRollerSpdCommand(0.5);
        }

    public Command runRollersOutCommand() {
        SmartDashboard.putString("rollers state", "running out");
        return changeRollerSpdCommand(-0.8);
    }

    public Command runRollersOutSlowCommand() {
        SmartDashboard.putString("rollers state", "running out");
        return changeRollerSpdCommand(-0.5);
    }

    public Command stopClawCommand() {
        SmartDashboard.putString("claw state", "stopped");
        return run(this::stopClaw);
    }

    public Command autoIntakeCommand() {
        return runRollersInCommand().until(this::isCoralInClaw).andThen(runRollersOutSlowCommand()).withTimeout(0.2).finallyDo(this::stopRoller);
    }

    public Command stopRollerCommand() {
        SmartDashboard.putString("rollers state", "stopped");
        return run(this::stopRoller);
    }

    public Command moveClawToPositionCommand(double pos) {
        return run(() -> setTargetPos(pos));
    }

    public Command moveClawToIntakeCommand() {
        targetState = "intake";
        SmartDashboard.putString("claw state", "moving to " + targetState);
        return moveClawToPositionCommand(ClawConstants.minEncoderValue);
    }

    public Command moveClawToHorizontalCommand() {
        targetState = "horizontal";
        SmartDashboard.putString("claw state", "moving to " + targetState);
        return moveClawToPositionCommand(ClawConstants.horizontalEncoderValue);
    }

    public Command moveClawToL4Command() {
        targetState = "L4";
        SmartDashboard.putString("claw state", "moving to " + targetState);
        return moveClawToPositionCommand(ClawConstants.l4EncoderValue);
    }

    public boolean isAtTarget(){
        return Math.abs(getClawPos() - getTargetPos()) < ClawConstants.tolerance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Claw position", getClawPos());
        SmartDashboard.putNumber("Claw target", getTargetPos());
        if (isAtTarget()) {
            SmartDashboard.putString("claw state", "at target" + targetState);
            stopClaw();
        } else {
            SmartDashboard.putString("claw state", "moving to " + targetState);
            var output = pidController.calculate(clawEncoder.getPosition(), targetPos);
            moveMotor.setVoltage(output);
        }
    }

}
