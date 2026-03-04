// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightPivotConstants;

public class LimelightPivotSubsystem extends SubsystemBase {
  /** Creates a new servoMotorSubsystem. */
  private final Servo servoMotor;

  public LimelightPivotSubsystem() {
    servoMotor = new Servo(LimelightPivotConstants.servoMotorChannel);
  }

  public void setAngleTo90() {
    servoMotor.setAngle(LimelightPivotConstants.deployAngle);
  }

  public void setAngleTo0() {
    servoMotor.setAngle(LimelightPivotConstants.retractAngle);
  }

  public Command setAngleTo90Cmd() {
    Command cmd = runOnce(this::setAngleTo90);
    cmd.setName("setAngleTo90Cmd");
    return cmd;
  }

  public Command setAngleTo0Cmd() {
    Command cmd = runOnce(this::setAngleTo0);
    cmd.setName("setAngleTo0Cmd");
    return cmd;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("servoMotorAngle", servoMotor.getAngle());
    SmartDashboard.putNumber("servoMotorPosition", servoMotor.get());
    SmartDashboard.putNumber("servoMotorSpeed", servoMotor.getSpeed());
  }
}
