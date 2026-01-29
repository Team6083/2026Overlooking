// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private final SparkMax climberMotor = new SparkMax(5, MotorType.kBrushless);
  private final SparkClosedLoopController closedLoopController = climberMotor.getClosedLoopController();
  private static final double L1_POSITION = 0.0;
  private static final double L2_POSITION = 0.0;
  private static final double MANUAL_SPEED = 0.5;

  public ClimberSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.0)
        .i(0.0)
        .d(0.0)
        .outputRange(-1.0, 1.0);
    config
        .idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake)
        .smartCurrentLimit(0);
  }

  private void toLowRung() {
    closedLoopController.setSetpoint(L1_POSITION, ControlType.kPosition);
  }

  private void toMidRung() {
    closedLoopController.setSetpoint(L2_POSITION, ControlType.kPosition);
  }

  private void climbUp() {
    climberMotor.set(MANUAL_SPEED);
  }

  private void climbDown() {
    climberMotor.set(-MANUAL_SPEED);
  }

  private void stopClimb() {
    climberMotor.set(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
