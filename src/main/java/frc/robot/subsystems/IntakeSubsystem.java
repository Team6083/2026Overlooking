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
      IntakeConstants.pivotEncoderFullRange, IntakeConstants.pivotLeftExpectedZero);//
  private final DutyCycleEncoder pivotRightEncoder = new DutyCycleEncoder(IntakeConstants.pivotRightEncoderId,
      IntakeConstants.pivotEncoderFullRange, IntakeConstants.pivotRightExpectedZero);
  private final PIDController pivotFollowPIDController = new PIDController(0.05, 0, 0);

  public IntakeSubsystem() {
    pivotLeft.setInverted(false);
    pivotRight.setInverted(true);
    pivotRightEncoder.setInverted(true);
    pivotLeftEncoder.setInverted(false);
    pivotFollowPIDController.enableContinuousInput(0, 360);
  }

  public double getRightPos() {
    return pivotRightEncoder.get();
  }

  public double getLeftPos() {
    return pivotLeftEncoder.get();
  }

  public void intake() {
    intakeMotor.set(ControlMode.PercentOutput, IntakeConstants.intakeSpeed);
  }

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

  public void reverseIntake() {
    intakeMotor.set(ControlMode.PercentOutput, IntakeConstants.reverseIntakeSpeed);
  }

  public void stopIntake() {
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public void retract() {
    if (pivotRightEncoder.get() - pivotLeftEncoder.get() <= 5) {
      pivotLeft.set(ControlMode.PercentOutput, IntakeConstants.reversePivotSpeed);
      double pivotRightSpeed = pivotFollowPIDController.calculate(pivotRightEncoder.get(),
          pivotLeftEncoder.get());
      pivotRight.set(ControlMode.PercentOutput, -pivotRightSpeed);
    } else if (pivotRightEncoder.get() - pivotLeftEncoder.get() >= 5) {
      pivotLeft.set(ControlMode.PercentOutput, 0);
      pivotRight.set(ControlMode.PercentOutput, IntakeConstants.reversePivotSpeed);
    }

  }

  public void deploy() {
    if (pivotRightEncoder.get() - pivotLeftEncoder.get() <= 5) {
      pivotLeft.set(ControlMode.PercentOutput, IntakeConstants.pivotSpeed);
      double pivotRightSpeed = pivotFollowPIDController.calculate(pivotLeftEncoder.get(),
          pivotRightEncoder.get());
      pivotRight.set(ControlMode.PercentOutput, -pivotRightSpeed);
    } else if (pivotRightEncoder.get() - pivotLeftEncoder.get() >= 5) {
      pivotLeft.set(ControlMode.PercentOutput, 0);
      pivotRight.set(ControlMode.PercentOutput, IntakeConstants.pivotSpeed);
    }
  }

  public Command deployRightintakeCmd() {
    Command cmd = runEnd(this::rightPivotDeploy, this::stopRotate);
    cmd.setName("deployRightIntakeCmd");
    return cmd;
  }

  public Command retractRightintaleCmd() {
    Command cmd = runEnd(this::rightPivotRetract, this::stopRotate);
    cmd.setName("restractrightCmd");
    return cmd;
  }

  public void deployRightintake() {
    pivotRight.set(ControlMode.PercentOutput, IntakeConstants.pivotSpeed);
  }

  public Command deployLeftintakeCmd() {
    Command cmd = runEnd(this::leftPivotDeploy, this::stopRotate);
    cmd.setName("deployLeftIntakeCmd");
    return cmd;
  }

  public Command restractLeftintakeCmd() {
    Command cmd = runEnd(this::leftPivotRetract, this::stopRotate);
    cmd.setName("restractLeftIntakeCmd");
    return cmd;
  }

  public void deployLeftintake() {
    pivotLeft.set(ControlMode.PercentOutput, IntakeConstants.pivotSpeed);
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
    Command cmd = runEnd(this::deploy, this::stopRotate);
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

    SmartDashboard.putNumber("pivotLeftAbsolutePosition", pivotLeftEncoder.get());
    SmartDashboard.putNumber("pivotRightAbsolutePosition", pivotRightEncoder.get());
    SmartDashboard.putBoolean("intakeEncoderConnected", pivotLeftEncoder.isConnected());
    SmartDashboard.putData(pivotFollowPIDController);
  }
}