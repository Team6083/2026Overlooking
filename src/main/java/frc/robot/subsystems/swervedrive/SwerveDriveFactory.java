// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;

public class SwerveDriveFactory {
  public enum SwerveImplementation {
    YAGSL,
    WPILIB
  }

  public enum RobotVariant {
    COMPETITION,
    CHASSIS
  }

  public static SwerveDrive createSwerveDrive(SwerveImplementation type, RobotVariant variant) {
    String swerveConfigDirName = switch (variant) {
      case COMPETITION -> "swerve/competition";
      case CHASSIS -> "swerve/chassis";
    };

    return switch (type) {
      case YAGSL -> new YagslSwerve(new File(Filesystem.getDeployDirectory(), swerveConfigDirName));
      case WPILIB -> new WpilibSwerveDrive();
    };
  }
}