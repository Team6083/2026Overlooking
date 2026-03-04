// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.lib.TagTracking;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final SparkMax shooterMotor = new SparkMax(ShooterConstants.shooterMotorID, MotorType.kBrushless);
  private final Encoder shooterEncoder = new Encoder(ShooterConstants.encoderChannelA,
      ShooterConstants.encoderChannelB);
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ShooterConstants.feedforwardKs,
      ShooterConstants.feedforwardKv, ShooterConstants.feedforwardKa);

  private final TagTracking tagTracking;
  private final Debouncer targetDebouncer = new Debouncer(0.2);
  private double distance;
  private double targetVelocity;

  public ShooterSubsystem(TagTracking tagTracking) {
    this.tagTracking = tagTracking;
    shooterEncoder.setDistancePerPulse((double) 1 / 2048);
  }

  private void setShooterVoltage(double voltage) {
    shooterMotor.setVoltage(voltage);
  }

  private double getShooterTargetSpeed() {
    boolean isTargetValid = targetDebouncer.calculate(tagTracking.hasTarget() && tagTracking.isHubTag());
    if (isTargetValid) {
      distance = Math.sqrt(Math.pow(tagTracking.getTx(), 2) + Math.pow(tagTracking.getTy(), 2));
      targetVelocity = 2588 * Math.exp(0.00431 * distance);
      return targetVelocity;
    } else {
      return ShooterConstants.targetVelocity;
    }
  }

  public void shoot() {
    double targetVelocity = getShooterTargetSpeed();
    double feedforwardVoltage = feedforward.calculate(targetVelocity);
    setShooterVoltage(feedforwardVoltage);
  }

  private void stopShooter() {
    shooterMotor.setVoltage(0);
  }

  private double getShooterVelocity() {
    return shooterEncoder.getRate() * 60;
  }

  public boolean isShooterAtSpeed() {
    return getShooterVelocity() >= getShooterTargetSpeed();
  }

  public Command shootCmd() {
    Command cmd = runEnd(this::shoot, this::stopShooter);
    cmd.setName("shootCmd");
    return cmd;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("isShooterAtSpeed", isShooterAtSpeed());
    SmartDashboard.putNumber("shooterVelocity", getShooterVelocity());
    SmartDashboard.putNumber("shooterMotorVoltage", shooterMotor.get() * shooterMotor.getBusVoltage());
    SmartDashboard.putData("ShooterSubsystem", this);
  }
}
