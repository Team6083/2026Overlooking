// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final VictorSPX intakeMotor = new VictorSPX(IntakeConstants.intakeMotorId);
  private final VictorSPX pivotLeft = new VictorSPX(IntakeConstants.pivotLeftId);
  private final VictorSPX pivotRight = new VictorSPX(IntakeConstants.pivotRightId);

  private final DutyCycleEncoder pivotLeftEncoder = new DutyCycleEncoder(IntakeConstants.pivotLeftEncoderId,
      IntakeConstants.pivotEncoderFullRange, IntakeConstants.pivotLeftExpectedZero);
  private final DutyCycleEncoder pivotRightEncoder = new DutyCycleEncoder(IntakeConstants.pivotRightEncoderId,
      IntakeConstants.pivotEncoderFullRange, IntakeConstants.pivotRightExpectedZero);

  private final PIDController pivotPIDController = new PIDController(0.1, 0, 0);
  private double pivotRightSpeed = 0;

  public IntakeSubsystem() {
    pivotLeft.setInverted(false);
    pivotRight.setInverted(true);
  }

  public void intake() {
    intakeMotor.set(ControlMode.PercentOutput, IntakeConstants.intakeSpeed);
  }

  public void leftPivot() {
    pivotLeft.set(ControlMode.PercentOutput, 0.1);
  }

  public void rightPivot(){
    pivotRight.set(ControlMode.PercentOutput, 0.1);
  }

  public void reverseIntake() {
    intakeMotor.set(ControlMode.PercentOutput, IntakeConstants.reverseIntakeSpeed);
  }

  public void stopIntake() {
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public void retract() {
    pivotLeft.set(ControlMode.PercentOutput, IntakeConstants.reversePivotSpeed);
    pivotRight.set(ControlMode.PercentOutput, IntakeConstants.reversePivotSpeed);
  }

  public void deployintake() {
    pivotLeft.set(ControlMode.PercentOutput, IntakeConstants.pivotSpeed);
    pivotRightSpeed = pivotPIDController.calculate(getPivotRightAbsolutePosition(), getPivotLeftAbsolutePosition());
    pivotRight.set(ControlMode.PercentOutput, pivotRightSpeed);
  }

  public void stopRotate() {
    pivotLeft.set(ControlMode.PercentOutput, 0);
    pivotRight.set(ControlMode.PercentOutput, 0);
  }

  public double getPivotLeftAbsolutePosition() {
    return pivotLeftEncoder.get();
  }

  public double getPivotRightAbsolutePosition() {
    return pivotRightEncoder.get();
  }

  public Command intakeCmd() {
    Command cmd = runEnd(this::intake, this::stopIntake);
    cmd.setName("startIntakeCmd");
    return cmd;
  }

  public Command reverseIntakeCmd() {
    Command cmd = runEnd(this::reverseIntake, this::stopIntake);
    cmd.setName("startReverseIntakeCmd");
    return cmd;
  }

  public Command manualRetractCmd() {
    Command cmd = runEnd(this::retract, this::stopRotate);
    cmd.setName("manualRetractCmd");
    return cmd;
  }

  public Command manualDeployIntakeCmd() {
    Command cmd = runEnd(this::deployintake, this::stopRotate);
    cmd.setName("manualDeployIntakeCmd");
    return cmd;
  }

  public Command deployIntakeCmd() {
    Command cmd = manualDeployIntakeCmd()
        .until(() -> getPivotLeftAbsolutePosition() > IntakeConstants.pivotDeployStopPosition||getPivotLeftAbsolutePosition() > IntakeConstants.pivotDeployStopPosition);
    cmd.setName("deployIntakeCmd");
    return cmd;
  }

  public Command retractIntakeCmd() {
    Command cmd = manualRetractCmd()
        .until(() -> getPivotLeftAbsolutePosition() < IntakeConstants.pivotRetractPosition);
    cmd.setName("retractIntakeCmd");
    return cmd;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("intakeMotorVoltage", intakeMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("pivotLeftMotorVoltage", pivotLeft.getMotorOutputVoltage());
    SmartDashboard.putNumber("pivotRightMotorVoltage", pivotRight.getMotorOutputVoltage());

    SmartDashboard.putNumber("pivotLeftAbsolutePosition", getPivotLeftAbsolutePosition());
    SmartDashboard.putNumber("pivotRightAbsolutePosition", getPivotRightAbsolutePosition());
    SmartDashboard.putBoolean("intakeEncoderConnected", pivotLeftEncoder.isConnected());
    SmartDashboard.putData(pivotPIDController);
  }
}