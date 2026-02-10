// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import swervelib.encoders.SparkMaxEncoderSwerve;


/** Add your docs here. */
public class auto {

    RobotConfig config ;
    
    public static void configureAutoBuilder(SwerveDrive drivetrain) {

          try {

            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                drivetrain::getPose2d,                // 現在位置
                drivetrain::resetPose,              // 重設位置
                drivetrain::getRobotRelativeSpeeds, // 現在速度
                
                (speeds, feedforwards) -> drivetrain.drive(speeds),
                
                new PPHolonomicDriveController(
                    new PIDConstants(5.75, 0.0, 0.75), 
                    new PIDConstants(4.5, 0.0, 0.5)
                ),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                },
                drivetrain 

            );

        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
