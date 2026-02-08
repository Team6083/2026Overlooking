// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
  }

  private final VictorSPX shooterMotor = new VictorSPX(ShooterConstants.shooterMotorID);
  private final Encoder shooterEncoder = new Encoder(ShooterConstants.encoderChannelA,
      ShooterConstants.encoderChannelB);
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ShooterConstants.feedforwardKs,
      ShooterConstants.feedforwardKv, ShooterConstants.feedforwardKa);

  private void shoot() {
    double targetVelocity = ShooterConstants.targetVelocity;
    double feedforwardVoltage = feedforward.calculate(targetVelocity);
    shooterMotor.set();
  }

  private void stopShooter() {
    shooterMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
