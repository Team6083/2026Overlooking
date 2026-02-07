// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;

public class autoSubsystem extends SubsystemBase {
  /** Creates a new autoSubsystem. */
  public autoSubsystem() {

   public static void setupAuto(DriveInterface drive, Subsystem driveSubsystem) {
        
        try {
            RobotConfig config = RobotConfig.fromGUISettings();
          } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

}
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
