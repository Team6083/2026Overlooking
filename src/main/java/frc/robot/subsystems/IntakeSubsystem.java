// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final SparkMax intakeMotor = new SparkMax(IntakeConstants.intakeMotorId, MotorType.kBrushless);
  private final SparkMax pivotLeft = new SparkMax(IntakeConstants.pivotLeftId, MotorType.kBrushless);
  private final SparkMax pivotRight = new SparkMax(IntakeConstants.pivotRightId, MotorType.kBrushless);
  private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(IntakeConstants.pivotEncoderId);

  public IntakeSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(false)
        .smartCurrentLimit(30)
        .idleMode(IdleMode.kBrake);

    intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pivotLeft.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pivotRight.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void intake() {
    intakeMotor.set(IntakeConstants.intakeSpeed);
  }

  public void reverseIntake() {
    intakeMotor.set(IntakeConstants.reverseIntakeSpeed);
  }

  public void stopIntake() {
    intakeMotor.set(IntakeConstants.stopIntakeSpeed);
  }

  public void rotateUp() {
    pivotLeft.set(IntakeConstants.pivotSpeed);
    pivotRight.set(-IntakeConstants.pivotSpeed);
  }

  public void rotateDown() {
    pivotLeft.set(IntakeConstants.reversePivotSpeed);
    pivotRight.set(-IntakeConstants.reversePivotSpeed);
  }

  public void stopRotate() {
    pivotLeft.set(IntakeConstants.stopPivotSpeed);
    pivotRight.set(IntakeConstants.stopPivotSpeed);
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
    SmartDashboard.putNumber("intakeAbsolutePosition", getPivotAbsolutePosition());
    SmartDashboard.putBoolean("intakeEncoderConnected", pivotEncoder.isConnected());
  }
}