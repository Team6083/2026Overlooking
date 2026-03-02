// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class servoMotorSubsystem extends SubsystemBase {
  /** Creates a new servoMotorSubsystem. */
  private final Servo servoMotor;

  public servoMotorSubsystem() {
    servoMotor = new Servo(0);
  }

  public void setAngleTo90() {
    servoMotor.setAngle(90);
  }

  public void setAngleTo0() {
    servoMotor.setAngle(0);
  }

  public Command setAngleTo0Cmd() {
    Command cmd = runOnce(this::setAngleTo0);
    cmd.setName("setAngleTo0Cmd");
    return cmd;
  }

  public Command setAngleTo90Cmd() {
    Command cmd = runOnce(this::setAngleTo90);
    cmd.setName("setAngleTo90Cmd");
    return cmd;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
