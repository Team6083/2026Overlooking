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
  private final Servo limelightPivot;

  public LimelightPivotSubsystem() {
    limelightPivot = new Servo(LimelightPivotConstants.servoMotorChannel);
  }

  public void deployLimelightPivot() {
    limelightPivot.setAngle(LimelightPivotConstants.deployAngle);
  }

  public void retractLimelightPivot() {
    limelightPivot.setAngle(LimelightPivotConstants.retractAngle);
  }

  public Command deployLimelightPivotCmd() {
    Command cmd = runOnce(this::deployLimelightPivot);
    cmd.setName("deployLimelightPivotCmd");
    return cmd;
  }

  public Command retractLimelightPivotCmd() {
    Command cmd = runOnce(this::retractLimelightPivot);
    cmd.setName("retractLimelightPivotCmd");
    return cmd;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("servoMotorAngle", limelightPivot.getAngle());
    SmartDashboard.putNumber("servoMotorPosition", limelightPivot.get());
  }
}
