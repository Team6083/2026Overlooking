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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final SparkMax intakeMotor = new SparkMax(IntakeConstants.intakeMotorId, MotorType.kBrushless);
  private final SparkMax rotateLeft = new SparkMax(IntakeConstants.rotateLeftId, MotorType.kBrushless);
  private final SparkMax rotateRight = new SparkMax(IntakeConstants.rotateRightId, MotorType.kBrushless);
  private final DutyCycleEncoder rotateEncoder = new DutyCycleEncoder(IntakeConstants.rotateEncoderId,
      IntakeConstants.rotateFullRange, IntakeConstants.rotateExpectedZero);

  public IntakeSubsystem() {
    SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    intakeMotorConfig
        .idleMode(IdleMode.kBrake);

    SparkMaxConfig pivotLeftConfig = new SparkMaxConfig();
    pivotLeftConfig
        .idleMode(IdleMode.kBrake);

    SparkMaxConfig pivotRightConfig = new SparkMaxConfig();
    pivotRightConfig
        .idleMode(IdleMode.kBrake)
        .inverted(true);

    intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rotateLeft.configure(pivotLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rotateRight.configure(pivotRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void intake() {
    intakeMotor.set(IntakeConstants.intakeSpeed);
  }

  public void reverseIntake() {
    intakeMotor.set(IntakeConstants.reverseIntakeSpeed);
  }

  public void stopIntake() {
    intakeMotor.set(0);
  }

  public void rotateUp() {
    rotateLeft.set(IntakeConstants.rotateSpeed);
    rotateRight.set(IntakeConstants.rotateSpeed);
  }

  public void rotateDown() {
    rotateLeft.set(IntakeConstants.reverseRotateSpeed);
    rotateRight.set(IntakeConstants.reverseRotateSpeed);
  }

  public void rotateStop() {
    rotateLeft.set(0);
    rotateRight.set(0);
  }

  public double getPivotAbsolutePosition() {
    return rotateEncoder.get();
  }

  public Command startIntakeCmd() {
    Command cmd = run(this::intake);
    cmd.setName("startIntakeCmd");
    return cmd;
  }

  public Command startReverseIntakeCmd() {
    Command cmd = run(this::reverseIntake);
    cmd.setName("startReverseIntakeCmd");
    return cmd;
  }

  public Command stopIntakeCmd() {
    Command cmd = runOnce(this::stopIntake);
    cmd.setName("StopIntakeCmd");
    return cmd;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("intakeMotorSpeed", intakeMotor.get());
    SmartDashboard.putNumber("intakeAbsolutePosition", getPivotAbsolutePosition());
    SmartDashboard.putBoolean("intakeEncoderConnected", rotateEncoder.isConnected());
  }
}