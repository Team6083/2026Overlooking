// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
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
  }

  private void toLowRung() {
    closedLoopController.setSetpoint(L1position, ControlType.kPosition);
  }

  private void toMidRung() {
    closedLoopController.setSetpoint(L2position, ControlType.kPosition);
  }

  private void climbUp() {
    climberMotor.set(motorSpeed);
  }

  private void climbDown() {
    climberMotor.set(-motorSpeed);
  }

  private void stopClimb() {
    climberMotor.set(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
