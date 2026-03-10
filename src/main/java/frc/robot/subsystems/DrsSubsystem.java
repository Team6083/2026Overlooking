// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrsConstants;

public class DrsSubsystem extends SubsystemBase {
  /** Creates a new servoMotorSubsystem. */
  private final Servo drs;

  public DrsSubsystem() {
    drs = new Servo(DrsConstants.servoMotorChannel);
  }

  public void downDrs() {
    drs.setPosition(DrsConstants.downPosition);
  }

  public void upDrs() {
    drs.setPosition(DrsConstants.upPosition);
  }

  public Command downDrsCmd() {
    Command cmd = runOnce(this::downDrs);
    cmd.setName("downDrsCmd");
    return cmd;
  }

  public Command upDrsCmd() {
    Command cmd = runOnce(this::upDrs);
    cmd.setName("upDrsCmd");
    return cmd;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("servoMotorAngle", drs.getAngle());
    SmartDashboard.putNumber("servoMotorPosition", drs.get());
  }
}
