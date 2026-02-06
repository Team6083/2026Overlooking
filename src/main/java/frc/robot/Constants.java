// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static final class VisionConstants {
    public static final double yawP = 0.05;
    public static final double yawTolerance = 1.5; 
    public static final double maxYawOutput = 0.5; 
    public static final double distP = 0.5;        
    public static final double idealDistance = 1.2; 
    public static final double distanceTolerance = 0.15; 
    public static final double maxForwardSpeed = 0.4;
    public static final int maxLostFrames = 10;
    public static final String limelightName = "limelight";
  }
}