// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TestElevatorConstants;

public class TestElevator extends SubsystemBase {
  private SparkMax motor1;
  private SparkMax motor2;
  /** Creates a new TestElevator. */
  public TestElevator() {
    motor1 = new SparkMax(TestElevatorConstants.motor1port, MotorType.kBrushless);
    motor2 = new SparkMax(TestElevatorConstants.motor2port, MotorType.kBrushless);
    SparkBaseConfig motor1Config = new SparkMaxConfig().idleMode(IdleMode.kCoast);
    SparkBaseConfig motor2Config = new SparkMaxConfig().follow(TestElevatorConstants.motor1port, true).idleMode(IdleMode.kCoast);
    motor2.configure(motor2Config, null, null);
    motor1.configure(motor1Config, null, null);
  }

  public void moveElevator(double speed) {
    motor1.set(speed);
  }
  
  public void stopElevator() {
    motor1.set(0);
  }

  public Command moveElevatorCommand(double speed) {
    return run(() -> moveElevator(speed));
  }

  public Command stopElevatorCommand() {
    return run(this::stopElevator);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
