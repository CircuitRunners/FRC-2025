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
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.utils.simulation.SimulationMechPoseUtil;
import java.util.function.Supplier;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

public class Elevator extends SubsystemBase {

    // Standard classes for controlling our elevator
    private final ProfiledPIDController m_controller = new ProfiledPIDController(
            ElevatorConstants.kElevatorKp,
            ElevatorConstants.kElevatorKi,
            ElevatorConstants.kElevatorKd,
            new TrapezoidProfile.Constraints(20, 20.0));
    ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
            ElevatorConstants.kElevatorkS,
            ElevatorConstants.kElevatorkG,
            ElevatorConstants.kElevatorkV,
            ElevatorConstants.kElevatorkA);
    private final SparkMax m_motor = new SparkMax(ElevatorConstants.kMotorPort, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();

    // SIMULATION OBJECTS!!! DO NOT TOUCH

    // Simulation classes help us simulate what's going on, including gravity.
    // This gearbox represents a gearbox containing 2 Rev NEO motors.
    private final DCMotor m_elevatorGearbox = DCMotor.getNEO(2);
    private final ElevatorSim m_elevatorSim = new ElevatorSim(
            m_elevatorGearbox,
            ElevatorConstants.kElevatorGearing,
            ElevatorConstants.kCarriageMass,
            ElevatorConstants.kElevatorDrumRadius,
            ElevatorConstants.kMinElevatorHeightMeters,
            ElevatorConstants.kMaxElevatorHeightMeters,
            true,
            0);
    private final SparkMaxSim m_motorSim = new SparkMaxSim(m_motor, m_elevatorGearbox);
    private final SparkRelativeEncoderSim m_encoderSim = m_motorSim.getRelativeEncoderSim();

    // SIMULATION OBJECTS!!! DO NOT TOUCH

    /** Subsystem constructor. */
    public Elevator() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.encoder.positionConversionFactor(ElevatorConstants.kElevatorEncoderDistPerPulse);
        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // Publish Mechanism2d to SmartDashboard
        // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim

        // SIMULATION BATTERY
        SimulatedBattery.addElectricalAppliances(getCurrent());
        // SIMULATION BATTERY
    }

    /**
     * Periodic function to run the elevator closed control loop. Calculates the output of the PID controller and the
     * feedforward based on the controller set goal, and sets the motor voltage.
     *
     * @see ProfiledPIDController
     * @see ElevatorFeedforward
     */
    @Override
    public void periodic() {
        double pidOutput = m_controller.calculate(m_encoder.getPosition());
        SmartDashboard.putNumber("PID Output", pidOutput);
        double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);
        SmartDashboard.putNumber("Feedforward Output", feedforwardOutput);
        m_motor.setVoltage(pidOutput + feedforwardOutput);
    }

    /**
     * set goal for elevator PID controller.
     *
     * @param goal the position to maintain
     */
    public void setGoal(double goal, boolean isInches) {
        if (isInches) {
            goal = Units.inchesToMeters(goal);
        }
        m_controller.setGoal(goal);

        // With the setpoint value we run PID control like normal
    }

    /**
     * Schedulable command to set the goal for the elevator
     *
     * @param goal
     * @param isInches
     * @return
     */
    public Command setGoalCommand(double goal, boolean isInches) {
        return runOnce(() -> {
            setGoal(goal, isInches);
        });
    }

    /** Stop the control loop and motor output. */
    public void stop() {
        m_controller.setGoal(0.0);
        m_motor.set(0.0);
    }

    // SIMULATION FUNCTIONS!!!!!!!!

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
     * Supplier function to get the pose3d of the first stage elevator to send to the simulation
     *
     * @return Supplier<Rotation3d> that supplies the current position of the first elevator stage in meters
     * @see SimulationMechPoseUtil
     */
    public Supplier<Pose3d> getStageOnePose() {
        return () -> new Pose3d(new Translation3d(0, 0, m_encoder.getPosition()), new Rotation3d());
    }

    /**
     * Supplier function to get the pose3d of the second stage elevator to send to the simulation. Uses the first stage
     * pose as a reference, and then adds the transform3d to move to the second stage (2x the height of the first stage
     * from cascade rigging)
     *
     * @return Supplier<Rotation3d> that supplies the current position of the seconds elevator stage in meters
     * @see SimulationMechPoseUtil
     */
    public Supplier<Pose3d> getStageTwoPose() {
        return () -> new Pose3d(new Translation3d(0, 0, m_encoder.getPosition()), new Rotation3d())
                .plus(new Transform3d(0, 0, m_encoder.getPosition(), new Rotation3d()));
    }

    /** Advance the simulation. */
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        m_elevatorSim.setInput(m_motorSim.getAppliedOutput()
                * SimulatedBattery.getBatteryVoltage().magnitude());

        // Next, we update it. The standard loop time is 20ms.
        m_elevatorSim.update(0.020);

        m_motorSim.iterate(
                m_elevatorSim.getVelocityMetersPerSecond(),
                SimulatedBattery.getBatteryVoltage().magnitude(),
                .02);

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        m_encoderSim.setPosition(m_elevatorSim.getPositionMeters());
        // SimBattery estimates loaded battery voltages    }
    }

    // SIMULATION FUNCTIONS!!!!!!!
}
