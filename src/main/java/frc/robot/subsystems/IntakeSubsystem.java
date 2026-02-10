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
  private final SparkMax pivotLeft = new SparkMax(IntakeConstants.pivotLeftId, MotorType.kBrushless);
  private final SparkMax pivotRight = new SparkMax(IntakeConstants.pivotRightId, MotorType.kBrushless);
  private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(IntakeConstants.pivotEncoderId,
      IntakeConstants.pivotFullRange, IntakeConstants.pivotExpectedZero);

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
    pivotLeft.configure(pivotLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pivotRight.configure(pivotRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

  public void retract() {
    pivotLeft.set(IntakeConstants.pivotSpeed);
    pivotRight.set(IntakeConstants.pivotSpeed);
  }

  public void deployintake() {
    pivotLeft.set(IntakeConstants.reversePivotSpeed);
    pivotRight.set(IntakeConstants.reversePivotSpeed);
  }

  public void stopRotate() {
    pivotLeft.set(0);
    pivotRight.set(0);
  }

  public double getPivotAbsolutePosition() {
    return pivotEncoder.get();
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
    SmartDashboard.putBoolean("intakeEncoderConnected", pivotEncoder.isConnected());
  }
}