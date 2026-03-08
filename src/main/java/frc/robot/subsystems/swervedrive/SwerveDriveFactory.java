// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.Constants.DriveBaseConstant;
import java.io.File;

public class SwerveDriveFactory {
  public enum SwerveImplementation {
    WPILIB
  }

  public enum RobotVariant {
    TEST,
    COMPETITION
  }

  public static SwerveDrive createSwerveDrive(SwerveImplementation type, RobotVariant variant) {
    String swerveConfigDirName = switch (variant) {
      case TEST -> "swerve/test";
      case COMPETITION -> "swerve/competition";
    };

    DriveBaseConstant driveBaseConstant = switch (variant) {
      case TEST -> Constants.TEST_CONFIG;
      case COMPETITION -> Constants.COMPETITION_CONFIG;
    };

    return switch (type) {
      // case YAGSL -> new YagslSwerve(new File(Filesystem.getDeployDirectory(),
      // swerveConfigDirName));
      case WPILIB -> new WpilibSwerveDrive(driveBaseConstant);
    };
  }
}