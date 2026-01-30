// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax intakeMotor = new SparkMax(10, MotorType.kBrushless);
  private final SparkMaxConfig config = new SparkMaxConfig();

  public IntakeSubsystem() {
    config.inverted(false)
        .smartCurrentLimit(30)
        .idleMode(IdleMode.kBrake);

    intakeMotor.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
  }

  public void intake() {
    intakeMotor.set(0.5);
  }

 private void reverseIntake() {
    intakeMotor.set(-0.5);
  }

  private void stopIntake() {
    intakeMotor.set(0);
  }

  private void rotateUp() {
    // Placeholder for rotate up logic
  }

  private void rotateDown() {
    // Placeholder for rotate down logic
  }

  private void stopRotate() {
    // Placeholder for stop rotate logic
  }

  @Override
  public void periodic() {
    // 這裡維持空白，或僅放 Sensor 數據更新
  }
}