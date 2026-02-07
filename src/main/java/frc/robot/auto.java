// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.swervedrive.SwerveDrive;

/** Add your docs here. */
public class auto {
    public static void configureAutoBuilder(SwerveDrive drivetrain) {
        try {
            // 1. 讀取 GUI 設定 (這行一定要包在 try-catch 裡)
            RobotConfig config = RobotConfig.fromGUISettings();
        }
            catch (Exception e) {
            // 2. 處理讀取設定時可能發生的例外
            e.printStackTrace();        
            }
        }
}
