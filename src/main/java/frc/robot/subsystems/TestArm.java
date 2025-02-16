// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TestArmConstants;

public class TestArm extends SubsystemBase {
  /** Creates a new TestArm. */
  private SparkMax motor1;
  private SparkMax motor2;

  private SparkAbsoluteEncoder encoder;
  public TestArm() {
    motor1 = new SparkMax(TestArmConstants.motor1port, MotorType.kBrushless);
    motor1.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), null, null);
    motor2 = new SparkMax(TestArmConstants.motor2port, MotorType.kBrushless);
    motor2.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast), null, null);
    encoder = motor1.getAbsoluteEncoder();

    SmartDashboard.putNumber("Arm Pivot encoder value (from motor)", 2.71828);
  }

  public void moveArm(double speed) {
    motor1.set(speed);
  }

  public void stopArm() {
    motor1.stopMotor();
  }

  public void moveOuttake(double speed) {
    motor2.set(speed);
  }

  public void stopOuttake() {
    motor2.stopMotor();
  }

  public Command moveArmCommand(double speed) {
    return run(() -> moveArm(speed));
  }

  public Command stopArmCommand() {
    return run(this::stopArm);
  }

  public Command moveOuttakeCommand(double speed) {
    return run(() -> moveOuttake(speed));
  }

  public Command stopOuttakeCommand() {
    return run(this::stopOuttake);
  }


  public double getPivotEncoderValue() {
    return encoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Pivot encoder value (from motor)", getPivotEncoderValue());
  }
}
