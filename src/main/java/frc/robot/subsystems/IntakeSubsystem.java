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
  private final SparkMax pivotLeft = new SparkMax(11, MotorType.kBrushless);
  private final SparkMax pivotRight = new SparkMax(12, MotorType.kBrushless);

  private final SparkMaxConfig config = new SparkMaxConfig();

  public IntakeSubsystem() {
    config.inverted(false)
        .smartCurrentLimit(30)
        .idleMode(IdleMode.kBrake);

    intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pivotLeft.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pivotRight.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void intake() {
    intakeMotor.set(0.5);
  }

  public void reverseIntake() {
    intakeMotor.set(-0.5);
  }

  public void stopIntake() {
    intakeMotor.set(0);
  }

  public void rotateUp() {
    pivotLeft.set(0.2);
    pivotRight.set(-0.2);
  }

  public void rotateDown() {
    pivotLeft.set(-0.2);
    pivotRight.set(0.2);
  }

  public void stopRotate() {
    pivotLeft.set(0);
    pivotRight.set(0);
  }

  @Override
  public void periodic() {
    
  }
}