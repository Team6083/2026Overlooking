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

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */

  private final SparkMax climberMotor = new SparkMax(ClimberConstants.motorId, MotorType.kBrushless);
  private final PIDController climberPID = new PIDController(
      ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD);
  private final DutyCycleEncoder climberEncoder = new DutyCycleEncoder(1, 360, 0);

  public ClimberSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake)
        .smartCurrentLimit(ClimberConstants.currentLimit);

    climberMotor.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    climberMotor.getEncoder().setPosition(0);
  }

  public void toLowRung() {
    climberPID.setSetpoint(ClimberConstants.L1position);
  }

  public void toMidRung() {
    climberPID.setSetpoint(ClimberConstants.L2position);
  }

  public void climbUp() {
    climberPID.setSetpoint(climberEncoder.get() + 10);
  }

  public void climbDown() {
    climberPID.setSetpoint(climberEncoder.get() - 10);
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
    Command cmd = this.runEnd(() -> toLowRung(), () -> climberMotor.set(0));
    cmd.setName("toLowRungCmd");
    return cmd;
  }

  public Command toMidRungCmd() {
    Command cmd = this.runEnd(() -> toMidRung(), () -> climberMotor.set(0));
    cmd.setName("toMidRungCmd");
    return cmd;
  }

  public Command climbUpCmd() {
    Command cmd = this.runEnd(() -> climbUp(), () -> climberMotor.set(0));
    cmd.setName("climbUpCmd");
    return cmd;
  }

  public Command climbDownCmd() {
    Command cmd = this.runEnd(() -> climbDown(), () -> climberMotor.set(0));
    cmd.setName("climbDownCmd");
    return cmd;
  }

  public Command stopClimbCmd() {
    Command cmd = this.runOnce(() -> this.climberMotor.set(0));
    cmd.setName("toHomeCmd");
    return cmd;
  }

  public Command resetEncoderCmd() {
    Command cmd = this.runOnce(() -> this.resetEncoder());
    cmd.setName("resetEncoderCmd");
    return cmd;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double measurement = MathUtil.clamp(climberEncoder.get(), -1.0, 1.0);
    climberMotor.set(climberPID.calculate(measurement));
    SmartDashboard.putNumber("Climber/Position", climberEncoder.get());
    SmartDashboard.putNumber("Climber/Target", climberPID.getSetpoint());
  }
}
