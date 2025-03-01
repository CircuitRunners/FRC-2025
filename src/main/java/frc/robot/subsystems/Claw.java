
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;


public class Claw extends SubsystemBase {
    
    private SparkMax moveMotor;
    private SparkMax roller1Motor;
    private SparkMax roller2Motor;
    private CANrange canRangeSensor;

    private PIDController pidController;
    private RelativeEncoder clawEncoder;

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

        double constP = 0.045; // proportional coefficient gain
        double constI = 0; // integral coefficient gain
        double constD = 0; // derivative coefficient gain

        pidController = new PIDController(constP, constI, constD);
        pidController.setTolerance(1);

        clawEncoder = moveMotor.getEncoder();
        targetPos = clawEncoder.getPosition(); // initialize targetPos so PID doesn't try calculating with a null value

    }

    public double getClawPos() {
        return clawEncoder.getPosition();
    }

    public double getTargetPos() {
        return targetPos;
    }

    public void setTargetPos(double desiredPos) {
        pidController.setSetpoint(desiredPos);
        targetPos = desiredPos;
    }

    public boolean isAtTarget(){
        return pidController.atSetpoint();
    }

    public void changeRollerSpd(double speed) {
        roller1Motor.set(speed);
    }

    public void stopRoller() {
        roller1Motor.stopMotor();
        roller2Motor.stopMotor();
    }

    public boolean isCoralInClaw() {
        return canRangeSensor.getDistance().getValueAsDouble() < ClawConstants.coralSensorRange;
    }

    public void stopClaw() {
        //setTargetPos(getClawPos()); // Hold claw position
        moveMotor.stopMotor();
    }



    // Commands

    public Command changeRollerSpdCommand(double speed) {
        return new FunctionalCommand(
            // onInitialize: set the rollers to the desired speed
            () -> changeRollerSpd(speed),
            // onExecute: keep the rollers running at the desired speed
            () -> changeRollerSpd(speed),
            // onEnd: when the command ends, stop the rollers
            interrupted -> changeRollerSpd(0),
            // isFinished: never finish on its own (so a deadline can end it)
            () -> false,
            // this subsystem is required
            this
        );
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
        SmartDashboard.putString("rollers state", "running out slow");
        return changeRollerSpdCommand(-0.5);
    }

    public Command stopClawCommand() {
        return run(() -> {
            SmartDashboard.putString("claw state", "stopped");
            stopClaw();
        });
    }

    public Command autoIntakeCommand() {
        return runRollersInCommand().until(this::isCoralInClaw).andThen(runRollersOutSlowCommand()).withTimeout(0.2).finallyDo(this::stopRoller);
    }

    public Command stopRollersCommand() {
        SmartDashboard.putString("rollers state", "stopped");
        return run(this::stopRoller);
    }

    public Command moveClawToPositionCommand(double pos) {
        return run(() -> setTargetPos(pos));
    }

    public void moveToPos(double targetPos) {
        this.targetPos = targetPos;
        pidController.setSetpoint(targetPos);
    }

    public Command moveWristCommand(double targetPosition) {
        return new MoveWristCommand(this, targetPosition);
    }

    public Command moveClawToIntakeCommand() {
        return moveWristCommand(ClawConstants.minEncoderValue);
    }

    public Command moveClawToHorizontalCommand() {
        return moveWristCommand(ClawConstants.horizontalEncoderValue);
    }

    public Command moveClawToL4Command() {
        return moveWristCommand(ClawConstants.l4EncoderValue);
    }


    public Command scoreL4() {
        return moveClawToHorizontalCommand().andThen(runRollersOutCommand().until(this::isAtTarget)); 
    }

    

    public Command runManualCommand(double speed){
        
        return run(() -> {
            SmartDashboard.putString("claw state", "moving");
            moveMotor.set(speed);});
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Claw position", clawEncoder.getPosition());
        SmartDashboard.putNumber("Claw target", getTargetPos());
        SmartDashboard.updateValues();
        double output = pidController.calculate(getClawPos(), this.targetPos);
        moveMotor.set(output);
        SmartDashboard.updateValues();
    }


    public static class MoveWristCommand extends Command {
        private final Claw claw;
        private final double targetPosition;

        public MoveWristCommand(Claw claw, double targetPosition) {
            this.claw = claw;
            this.targetPosition = targetPosition;
            addRequirements(claw);
        }

        @Override
        public void initialize() {
            claw.setTargetPos(targetPosition);
        }

        @Override
        public boolean isFinished() {
            return claw.isAtTarget();
        }
    }
}
