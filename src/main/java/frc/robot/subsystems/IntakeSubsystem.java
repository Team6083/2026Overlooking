// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
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
  private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(IntakeConstants.pivotEncoderId,
      IntakeConstants.pivotFullRange, IntakeConstants.pivotExpectedZero);

  public IntakeSubsystem() {
    intakeMotor.setInverted(IntakeConstants.intakeInverted);
    
    pivotLeft.setInverted(false);
    pivotRight.setInverted(true);
  }

  public void intake() {
    intakeMotor.set(ControlMode.PercentOutput, IntakeConstants.intakeSpeed);
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
    pivotRight.set(ControlMode.PercentOutput, IntakeConstants.pivotSpeed);
  }

  public void stopRotate() {
    pivotLeft.set(ControlMode.PercentOutput, 0);
    pivotRight.set(ControlMode.PercentOutput, 0);
  }

  public double getPivotAbsolutePosition() {
    return pivotEncoder.get();
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
        .until(() -> getPivotAbsolutePosition() > IntakeConstants.pivotDeployStopPosition);
    cmd.setName("deployIntakeCmd");
    return cmd;
  }

  public Command retractIntakeCmd() {
    Command cmd = manualRetractCmd()
        .until(() -> getPivotAbsolutePosition() < IntakeConstants.pivotRetractPosition);
    cmd.setName("retractIntakeCmd");
    return cmd;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("intake/motorVoltage", intakeMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("intake/pivotPositionDeg", getPivotAbsolutePosition());
    SmartDashboard.putBoolean("intake/encoderConnected", pivotEncoder.isConnected());
    SmartDashboard.putNumber("intake/pivotLeftVoltage", pivotLeft.getMotorOutputVoltage());
    SmartDashboard.putNumber("intake/pivotRightVoltage", pivotRight.getMotorOutputVoltage());
    SmartDashboard.putData("intake/subsystem", this);
  }
}