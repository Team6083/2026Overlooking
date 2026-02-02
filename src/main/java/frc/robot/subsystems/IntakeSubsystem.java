// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax intakeMotor = new SparkMax(10, MotorType.kBrushless);
  private final SparkMax pivotLeft = new SparkMax(11, MotorType.kBrushless);
  private final SparkMax pivotRight = new SparkMax(12, MotorType.kBrushless);
  private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(4);

  @SuppressWarnings("removal")
  public IntakeSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(false)
        .smartCurrentLimit(30)
        .idleMode(IdleMode.kBrake);

    intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    pivotLeft.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    pivotRight.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
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

  public void deployIntake() {
    rotateDown();
  }

  public void retractIntake() {
    rotateUp();
  }

  public double getPivotAbsolutePosition() {
    return pivotEncoder.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("intakemotorspeed", intakeMotor.get());
    SmartDashboard.putNumber("Pivot Absolute Pos", getPivotAbsolutePosition());
    SmartDashboard.putBoolean("Pivot Encoder Connected", pivotEncoder.isConnected());
  }
}
