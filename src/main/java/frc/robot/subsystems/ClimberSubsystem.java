// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

import java.util.function.BooleanSupplier;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */

  private final SparkMax climberMotor = new SparkMax(ClimberConstants.motorId, MotorType.kBrushless);
  private final PIDController climberPID = new PIDController(
      ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD);
  private final DutyCycleEncoder climberEncoder = new DutyCycleEncoder(ClimberConstants.encoderChannel, 360, 0);
  private final BooleanSupplier manualModeSupplier;

  public ClimberSubsystem(BooleanSupplier manualModeSupplier) {
    this.manualModeSupplier = manualModeSupplier;
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake)
        .smartCurrentLimit(ClimberConstants.currentLimit);

    climberMotor.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void toLowRung() {
    if (manualModeSupplier.getAsBoolean())
      return;
    climberPID.setSetpoint(ClimberConstants.L1position);
  }

  public void toMidRung() {
    if (manualModeSupplier.getAsBoolean())
      return;
    climberPID.setSetpoint(ClimberConstants.L2position);
  }

  public void climbUp() {
    if (manualModeSupplier.getAsBoolean()) {
      climberMotor.set(0.3);
    } else {
      climberPID.setSetpoint(climberEncoder.get() + 10);
    }

  }

  public void climbDown() {
    if (manualModeSupplier.getAsBoolean()) {
      climberMotor.set(-0.3);
    } else {
      climberPID.setSetpoint(climberEncoder.get() - 10);
    }

  }

  public void toHome() {
    climberPID.setSetpoint(0);
  }

  public void resetEncoder() {
    climberMotor.getEncoder().setPosition(0);
    climberPID.reset();
    climberPID.setSetpoint(0);

  }

  public Command toLowRungCmd() {
    Command cmd = this.runOnce(() -> toLowRung());
    cmd.setName("toLowRungCmd");
    return cmd;
  }

  public Command toMidRungCmd() {
    Command cmd = this.runOnce(() -> toMidRung());
    cmd.setName("toMidRungCmd");
    return cmd;
  }

  public Command climbUpCmd() {
    Command cmd = this.run(() -> climbUp()).finallyDo(() -> climberMotor.set(0));
    cmd.setName("climbUpCmd");
    return cmd;
  }

  public Command climbDownCmd() {
    Command cmd = this.run(() -> climbDown()).finallyDo(() -> climberMotor.set(0));
    cmd.setName("climbDownCmd");
    return cmd;
  }

  public Command stopClimbCmd() {
    Command cmd = this.runOnce(() -> climberMotor.set(0));
    cmd.setName("toHomeCmd");
    return cmd;
  }

  public Command resetEncoderCmd() {
    Command cmd = this.runOnce(() -> resetEncoder());
    cmd.setName("resetEncoderCmd");
    return cmd;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    boolean isManualMode = manualModeSupplier.getAsBoolean();
    if (isManualMode) {
      SmartDashboard.putString("climberStatus", "manualMode");
    } else {
      double currentPosition = climberEncoder.get();
      double pidOutput = climberPID.calculate(currentPosition);
      double limitedOutput = MathUtil.clamp(pidOutput, -0.5, 0.5);
      if (climberPID.atSetpoint()) {
        climberMotor.set(0);
      } else {
        climberMotor.set(limitedOutput);
      }

    }

    SmartDashboard.putNumber("climberPosition", climberEncoder.get());
    SmartDashboard.putNumber("climberTarget", climberPID.getSetpoint());
  }
}
