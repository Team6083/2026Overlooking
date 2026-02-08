// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivebase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
  /** Creates a new SwerveDrive. */
  public SwerveModule testModule = new SwerveModule(
      23, 24, 34, 0.928711, true, false);

  public SwerveDrive() {
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("angle", testModule.getAngleRadians());
    SmartDashboard.putData(testModule.pid);
    SmartDashboard.putNumber("motorOutput", testModule.turningMotor.get());
    // This method will be called once per scheduler run
  }
}
