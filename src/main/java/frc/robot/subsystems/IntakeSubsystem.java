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
      IntakeConstants.pivotEncoderFullRange, 327);
  private final DutyCycleEncoder pivotRightEncoder = new DutyCycleEncoder(IntakeConstants.pivotRightEncoderId,
      IntakeConstants.pivotEncoderFullRange, 132);

  private final PIDController pivotFollowPIDController = new PIDController(0.05, 0, 0);

  public IntakeSubsystem() {
    pivotLeft.setInverted(false);
    pivotRight.setInverted(true);
  }

  public double getRightPos() {
    return 360.0 - pivotRightEncoder.get(); 
  }

  public double getLeftPos() {
    return pivotLeftEncoder.get(); 
  }

  public void intake() {
    intakeMotor.set(ControlMode.PercentOutput, IntakeConstants.intakeSpeed);
  }

  public void leftPivotDeploy() {
    pivotLeft.set(ControlMode.PercentOutput, 0.1);
  }

  public void rightPivotDeploy() {
    pivotRight.set(ControlMode.PercentOutput, 0.1);
  }

  public void leftPivotRetract() {
    pivotLeft.set(ControlMode.PercentOutput, -0.1);
  }

  public void rightPivotRetract() {
    pivotRight.set(ControlMode.PercentOutput, -0.1);
  }

  public void reverseIntake() {
    intakeMotor.set(ControlMode.PercentOutput, IntakeConstants.reverseIntakeSpeed);
  }

  public void stopIntake() {
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public void retract() {
    pivotLeft.set(ControlMode.PercentOutput, IntakeConstants.reversePivotSpeed);
    double pivotRightSpeed = pivotFollowPIDController.calculate(pivotRightEncoder.get(),
        pivotLeftEncoder.get());
    pivotRight.set(ControlMode.PercentOutput, pivotRightSpeed);
  }

  public void deployintake() {
    pivotLeft.set(ControlMode.PercentOutput, IntakeConstants.pivotSpeed);
    double pivotRightSpeed = pivotFollowPIDController.calculate(pivotRightEncoder.get(),
        pivotLeftEncoder.get());
    pivotRight.set(ControlMode.PercentOutput, pivotRightSpeed);
  }

  public Command deployRightintakeCmd() {
    Command cmd = runEnd(this::deployRightintake, this::stopRotate);
    cmd.setName("deployRightIntakeCmd");
    return cmd;
  }

  public void deployRightintake() {
    pivotRight.set(ControlMode.PercentOutput, IntakeConstants.pivotSpeed);
  }

  public Command deployLeftintakeCmd() {
    Command cmd = runEnd(this::deployLeftintake, this::stopRotate);
    cmd.setName("deployLeftIntakeCmd");
    return cmd;
  }

  public void deployLeftintake() {
    pivotLeft.set(ControlMode.PercentOutput, -IntakeConstants.pivotSpeed);
  }

  public void stopRotate() {
    pivotLeft.set(ControlMode.PercentOutput, 0);
    pivotRight.set(ControlMode.PercentOutput, 0);
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
        .until(() -> getRightPos() >= IntakeConstants.pivotDeployStopPosition
            && getLeftPos() >= IntakeConstants.pivotDeployStopPosition);
    cmd.setName("deployIntakeCmd");
    return cmd;
  }

  public Command retractIntakeCmd() {
    Command cmd = manualRetractCmd()
        .until(() -> getLeftPos() <= IntakeConstants.pivotRetractPosition
            && getRightPos() <= IntakeConstants.pivotRetractPosition);
    cmd.setName("retractIntakeCmd");
    return cmd;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("intakeMotorVoltage", intakeMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("pivotLeftMotorVoltage", pivotLeft.getMotorOutputVoltage());
    SmartDashboard.putNumber("pivotRightMotorVoltage", pivotRight.getMotorOutputVoltage());

    SmartDashboard.putNumber("pivotLeftAbsolutePosition", getLeftPos());
    SmartDashboard.putNumber("pivotRightAbsolutePosition", getRightPos());
    SmartDashboard.putBoolean("intakeEncoderConnected", pivotLeftEncoder.isConnected());
    SmartDashboard.putData(pivotFollowPIDController);
  }
}