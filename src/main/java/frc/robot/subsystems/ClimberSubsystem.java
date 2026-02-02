// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;

import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  public enum ClimberAction{UP,DOWN,L1,L2}
  private final SparkMax climberMotor = new SparkMax(ClimberConstants.motorId, MotorType.kBrushless);
  private final SparkClosedLoopController closedLoopController = climberMotor.getClosedLoopController();
  private static final double L1position = 25;
  private static final double L2position = 50;
  private static final double motorSpeed = 0.5;

  public ClimberSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();

    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(ClimberConstants.kP)
        .i(ClimberConstants.kI)
        .d(ClimberConstants.kD)
        .outputRange(ClimberConstants.minOutput, ClimberConstants.maxOutput);
    config
        .idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake)
        .smartCurrentLimit(ClimberConstants.currentLimit);

    climberMotor.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void toLowRung() {
    closedLoopController.setSetpoint(L1position, ControlType.kPosition);
  }

  public void toMidRung() {
    closedLoopController.setSetpoint(L2position, ControlType.kPosition);
  }

  public void climbUp() {
    climberMotor.set(motorSpeed);
  }

  public void climbDown() {
    climberMotor.set(-motorSpeed);
  }

  public void stopClimb() {
    climberMotor.set(0);
  }

  public Command toLowRungCmd() {
    Command cmd = this.runEnd(() -> toLowRung(), () -> stopClimb());
    cmd.setName("toLowRungCmd");
    return cmd;
  }

  public Command toMidRungCmd() {
    Command cmd = this.runEnd(() -> toMidRung(), () -> stopClimb());
    cmd.setName("toMidRungCmd");
    return cmd;
  }

  public Command climbUpCmd() {
    Command cmd = this.runEnd(() -> climbUp(), () -> stopClimb());
    cmd.setName("climbUpCmd");
    return cmd;
  }

  public Command climbDownCmd() {
    Command cmd = this.runEnd(() -> climbDown(), () -> stopClimb());
    cmd.setName("climbDownCmd");
    return cmd;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
