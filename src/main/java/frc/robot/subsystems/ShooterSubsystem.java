// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final SparkMax shooterMotor = new SparkMax(ShooterConstants.shooterMotorID, MotorType.kBrushless);
  private final RelativeEncoder shooterEncoder;
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ShooterConstants.feedforwardKs,
      ShooterConstants.feedforwardKv, ShooterConstants.feedforwardKa);

  private double targetVelocity = 0;

  public ShooterSubsystem() {
    shooterEncoder = shooterMotor.getEncoder();
  }

  private void setShooterVoltage(double voltage) {
    shooterMotor.setVoltage(voltage);
  }

  public void shoot(double targetVelocity) {
    this.targetVelocity = targetVelocity;
    double feedforwardVoltage = feedforward.calculate(targetVelocity);
    setShooterVoltage(feedforwardVoltage);
  }

  public void stopShooter() {
    this.targetVelocity = 0;
    shooterMotor.setVoltage(0);
  }

  private double getShooterVelocity() {
    return shooterEncoder.getVelocity();
  }

  public boolean isShooterAtSpeed() {
    return getShooterVelocity() >= targetVelocity;
  }

  public Command shootCmd() {
    Command cmd = runEnd(() -> shoot(ShooterConstants.targetVelocity), this::stopShooter);
    cmd.setName("shootCmd");
    return cmd;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("shooter/shooterAtSpeed", isShooterAtSpeed());
    SmartDashboard.putNumber("shooter/shooterRPM", getShooterVelocity());
    SmartDashboard.putNumber("shooter/targetRPM", targetVelocity);
    SmartDashboard.putNumber("shooter/shooterVoltage", shooterMotor.getAppliedOutput() * shooterMotor.getBusVoltage());
    SmartDashboard.putData("shooter/subsystem", this);
  }
}