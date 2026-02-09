// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final VictorSPX shooterMotor = new VictorSPX(ShooterConstants.shooterMotorID);
  private final Encoder shooterEncoder = new Encoder(ShooterConstants.encoderChannelA,
      ShooterConstants.encoderChannelB);
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ShooterConstants.feedforwardKs,
      ShooterConstants.feedforwardKv, ShooterConstants.feedforwardKa);

  public ShooterSubsystem() {
    shooterEncoder.setDistancePerPulse((double) 1 / 2048);
  }

  private void setShooterVoltage(double voltage) {
    shooterMotor.set(ControlMode.PercentOutput, voltage / shooterMotor.getBusVoltage());
  }

  public void shoot() {
    double targetVelocity = ShooterConstants.targetVelocity;
    double feedforwardVoltage = feedforward.calculate(targetVelocity);
    setShooterVoltage(feedforwardVoltage);
  }

  public void stopShooter() {
    shooterMotor.set(ControlMode.PercentOutput, 0);
  }

  private double getShooterVelocity() {
    return shooterEncoder.getRate();
  }

  private boolean isShooterAtSpeed() {
    return getShooterVelocity() >= ShooterConstants.targetVelocity;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
