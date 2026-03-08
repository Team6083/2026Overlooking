// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.swervedrive.SwerveDrive;

/** Add your docs here. */
public class Auto {
  private static SwerveDrive swerveDrive;

  public static void configureAutoBuilder(SwerveDrive swerveDrive) {

    try {
      RobotConfig config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
          swerveDrive::getPose2d, // 現在位置
          swerveDrive::resetPose, // 重設位置
          swerveDrive::getRobotRelativeSpeeds, // 現在速度

          (speeds, feedforwards) -> swerveDrive.drive(speeds),

          new PPHolonomicDriveController(
              new PIDConstants(AutoConstants.kpTranslation, AutoConstants.kiTranslation, AutoConstants.kdTranslation),
              new PIDConstants(AutoConstants.kpRotation, AutoConstants.kiRotation, AutoConstants.kdRotation)),
          config,
          () -> {
            var alliance = DriverStation.getAlliance();
            return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
          },
          swerveDrive

      );

    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  public static Command findToPoseCmd(Pose2d targetPose, double goalEndState) {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      targetPose = FlippingUtil.flipFieldPose(targetPose);
    }
    return AutoBuilder.pathfindToPose(targetPose,
        new PathConstraints(4.9, 4, Math.PI * 4, Math.PI * 6),
        goalEndState);
  }

  public static Command findToPoseCmd(Pose2d targetPose, double goalEndState, double maxVel, double maxAccel) {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      targetPose = FlippingUtil.flipFieldPose(targetPose);
    }
    return AutoBuilder.pathfindToPose(targetPose,
        new PathConstraints(maxVel, maxAccel, Math.PI * 4, Math.PI * 6),
        goalEndState);
  }

  public static Command setCurrentPoseCmd(Pose2d currentPose2d) {
    final Pose2d currentPose;
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      currentPose = FlippingUtil.flipFieldPose(currentPose2d);
    } else {
      currentPose = currentPose2d;
    }
    Command cmd = Commands.run(() -> swerveDrive.resetPose(currentPose));
    return cmd;
  }
}
