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
  private final PIDController pivotFollowPIDController = new PIDController(IntakeConstants.pivotFollowKp,
      IntakeConstants.pivotFollowKi, IntakeConstants.pivotFollowKd);

  public IntakeSubsystem() {
    pivotLeft.setInverted(IntakeConstants.motorLeftInverted);
    pivotRight.setInverted(IntakeConstants.motorRightInverted);
    pivotLeftEncoder.setInverted(IntakeConstants.encoderLeftInverted);
    pivotRightEncoder.setInverted(IntakeConstants.encoderRightInverted);
    pivotFollowPIDController.enableContinuousInput(IntakeConstants.pivotFollowMinInput,
        IntakeConstants.pivotFollowMaxInput);
  }

  // Intake
  public void intake() {
    intakeMotor.set(ControlMode.PercentOutput, IntakeConstants.intakeSpeed);
  }

  public void reverseIntake() {
    intakeMotor.set(ControlMode.PercentOutput, IntakeConstants.reverseIntakeSpeed);
  }

  public void stopIntake() {
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  // Pivot
  public void leftPivotDeploy() {
    pivotLeft.set(ControlMode.PercentOutput, 0.3);
  }

  public void rightPivotDeploy() {
    pivotRight.set(ControlMode.PercentOutput, 0.3);
  }

  public void leftPivotRetract() {
    pivotLeft.set(ControlMode.PercentOutput, -0.3);
  }

  public void rightPivotRetract() {
    pivotRight.set(ControlMode.PercentOutput, -0.3);
  }

  public void stopRotate() {
    pivotLeft.set(ControlMode.PercentOutput, 0);
    pivotRight.set(ControlMode.PercentOutput, 0);
  }

  // Sync Pivot
  public void deploy() {
    runSyncPivot(IntakeConstants.pivotSpeed);
  }

  public void retract() {
    runSyncPivot(IntakeConstants.reversePivotSpeed);
  }

  private void runSyncPivot(double baseSpeed) {
    double leftPos = pivotLeftEncoder.get();
    double rightPos = pivotRightEncoder.get();
    double syncCorrection = pivotFollowPIDController.calculate(rightPos, leftPos);
    pivotLeft.set(ControlMode.PercentOutput, baseSpeed);
    pivotRight.set(ControlMode.PercentOutput, baseSpeed + syncCorrection);
  }

  public double getLeftPos() {
    return pivotLeftEncoder.get();
  }

  public double getRightPos() {
    return pivotRightEncoder.get();
  }

  // Commands
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

  public Command deployLeftIntakeCmd() {
    Command cmd = runEnd(this::leftPivotDeploy, this::stopRotate);
    cmd.setName("deployLeftIntakeCmd");
    return cmd;
  }

  public Command deployRightIntakeCmd() {
    Command cmd = runEnd(this::rightPivotDeploy, this::stopRotate);
    cmd.setName("deployRightIntakeCmd");
    return cmd;
  }

  public Command retractLeftIntakeCmd() {
    Command cmd = runEnd(this::leftPivotRetract, this::stopRotate);
    cmd.setName("retractLeftIntakeCmd");
    return cmd;
  }

  public Command retractRightIntakeCmd() {
    Command cmd = runEnd(this::rightPivotRetract, this::stopRotate);
    cmd.setName("retractRightIntakeCmd");
    return cmd;
  }

  public Command syncDeployIntakeCmd() {
    Command cmd = runEnd(this::deploy, this::stopRotate);
    cmd.setName("syncDeployIntakeCmd");
    return cmd;
  }

  public Command syncRetractIntakeCmd() {
    Command cmd = runEnd(this::retract, this::stopRotate);
    cmd.setName("syncRetractIntakeCmd");
    return cmd;
  }

  public Command deployIntakeCmd() {
    Command cmd = syncDeployIntakeCmd()
        .until(() -> getRightPos() >= IntakeConstants.pivotDeployStopPosition
            && getLeftPos() >= IntakeConstants.pivotDeployStopPosition);
    cmd.setName("deployIntakeCmd");
    return cmd;
  }

  public Command retractIntakeCmd() {
    Command cmd = syncRetractIntakeCmd()
        .until(() -> getLeftPos() <= IntakeConstants.pivotRetractStopPosition
            && getRightPos() <= IntakeConstants.pivotRetractStopPosition);
    cmd.setName("retractIntakeCmd");
    return cmd;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("intakeMotorVoltage", intakeMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("pivotLeftMotorVoltage", pivotLeft.getMotorOutputVoltage());
    SmartDashboard.putNumber("pivotRightMotorVoltage", pivotRight.getMotorOutputVoltage());
    SmartDashboard.putNumber("pivotLeftAbsolutePosition", pivotLeftEncoder.get());
    SmartDashboard.putNumber("pivotRightAbsolutePosition", pivotRightEncoder.get());
    SmartDashboard.putBoolean("pivotLeftEncoderConnected", pivotLeftEncoder.isConnected());
    SmartDashboard.putBoolean("pivotRightEncoderConnected", pivotRightEncoder.isConnected());
    SmartDashboard.putData(pivotFollowPIDController);
  }
}