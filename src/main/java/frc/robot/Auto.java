// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.swervedrive.SwerveDrive;

/** Add your docs here. */
public class Auto {

  RobotConfig config;

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

  // public Command followPathCommand(SwerveDrive swerveDrive, String pathName) {
  // try{
  // PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

  // return new FollowPathCommand(
  // path,
  // swerveDrive::getPose2d,
  // swerveDrive::getRobotRelativeSpeeds,
  // swerveDrive::drive(ChassisSpeeds speeds),
  // new PPHolonomicDriveController(
  // new PIDConstants(5.0, 0.0, 0.0),
  // new PIDConstants(5.0, 0.0, 0.0)
  // ),
  // Constants.robotConfig,
  // () -> {
  // var alliance = DriverStation.getAlliance();
  // if(alliance.isPresent() ){
  // return alliance.get() == DriverStation.Alliance.Red;
  // }

  // }

  //

}
