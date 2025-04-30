// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import java.util.function.Supplier;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

public class Claw extends SubsystemBase {

    private double m_armKp = ArmConstants.kDefaultArmKp;

    // Standard classes for controlling our elevator
    private final PIDController m_controller = new PIDController(m_armKp, 0, 0);
    private final SparkMax m_motor = new SparkMax(ArmConstants.kMotorPort, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();

    // SIMULATION OBJECTS!! DO NOT TOUCH
    private final DCMotor m_armGearbox = DCMotor.getNEO(1);
    private final SingleJointedArmSim m_armSim = new SingleJointedArmSim(
            m_armGearbox,
            ArmConstants.kArmReduction,
            SingleJointedArmSim.estimateMOI(ArmConstants.kArmLength, ArmConstants.kArmMass),
            ArmConstants.kArmLength,
            ArmConstants.kMinAngleRads,
            ArmConstants.kMaxAngleRads,
            true,
            0);
    private final SparkMaxSim m_motorSim = new SparkMaxSim(m_motor, m_armGearbox);
    private final SparkRelativeEncoderSim m_encoderSim = m_motorSim.getRelativeEncoderSim();
    // SIMULATION OBJECTS!! DO NOT TOUCH

    /** Subsystem constructor. */
    public Claw() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.encoder.positionConversionFactor(ArmConstants.kArmEncoderDistPerPulse);
        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // SIMULATION BATTERY
        SimulatedBattery.addElectricalAppliances(getCurrent());
        // SIMULATION BATTERY

    }

    /**
     * Periodic function called to run the claw arm control loop Calculates the PID output using the current arm encoder
     * position and the arm setpoint, and sets the motor voltage
     *
     * @see PIDController
     */
    @Override
    public void periodic() {
        double pidOutput = m_controller.calculate(m_encoder.getPosition());
        SmartDashboard.putNumber("PID Output", pidOutput);
        m_motor.setVoltage(pidOutput);
    }

    /**
     * set goal for elevator PID controller.
     *
     * @param goal the position to maintain
     */
    public void setSetpoint(double goal) {
        goal = Units.degreesToRadians(goal);
        m_controller.setSetpoint(goal);

        // With the setpoint value we run PID control like normal
    }

    // Schedulable command to set the setpoint for the elevator
    public Command setGoalCommand(double setpoint) {
        return runOnce(() -> {
            setSetpoint(setpoint);
        });
    }

    /** Stop the control loop and motor output. */
    public void stop() {
        m_controller.setSetpoint(0.0);
        m_motor.set(0.0);
    }

    // SIMULATION FUNCTIONS!!!!!!!!

    public void simulationPeriodic() {
        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        m_armSim.setInput(m_motorSim.getAppliedOutput()
                * SimulatedBattery.getBatteryVoltage().magnitude());

        // Next, we update it. The standard loop time is 20ms.
        m_armSim.update(0.020);

        // Iterate SparkMax simulation with arm simulation velocity and simulated battery voltage
        m_motorSim.iterate(
                m_armSim.getVelocityRadPerSec(),
                SimulatedBattery.getBatteryVoltage().magnitude(),
                .02);

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        m_encoderSim.setPosition(m_armSim.getAngleRads());
        // SimBattery estimates loaded battery voltages    }
    }

    /**
     * Supplier function to get the current applied to the motor for use in the simulated battery current draw
     * calculation
     *
     * @return Supplier<Current> that returns the current applied to the motor
     * @see SimulatedBattery
     */
    public Supplier<Current> getCurrent() {
        return () -> Current.ofBaseUnits(m_motor.getOutputCurrent(), Amps);
    }

    /**
     * Supplier function to get the current position of the arm in radians to send to the simulation
     *
     * @return Supplier<Rotation3d> that returns the current position of the arm in radians
     * @see SimulationMechPoseUtil
     */
    public Supplier<Rotation3d> getPose() {
        return () -> new Rotation3d(0, -m_encoder.getPosition(), 0);
    }

    // SIMULATION FUNCTIONS!!!!!!!!

}
