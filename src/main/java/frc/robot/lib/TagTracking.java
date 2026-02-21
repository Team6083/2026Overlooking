// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class TagTracking {
  private final NetworkTable limelightTable;
  private boolean disabled = false;

  public TagTracking() {
    this("limelight");
  }

  public TagTracking(String name) {
    limelightTable = NetworkTableInstance.getDefault().getTable(name);
  }

  public double getTv() {
    return limelightTable.getEntry("tv").getDouble(0);
  }

  public boolean hasTarget() {
    return getTv() == 1;
  }

  public double getTid() {
    return hasTarget() ? limelightTable.getEntry("tid").getDouble(0) : 0;
  }

  public double getTx() {
    return hasTarget() ? limelightTable.getEntry("tx").getDouble(0) : 0;
  }

  public double getTy() {
    return hasTarget() ? limelightTable.getEntry("ty").getDouble(0) : 0;
  }

  public double[] getTargetPoseRobotSpace() {
    return limelightTable.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
  }

  public double get3dTx() {
    return hasTarget() ? getTargetPoseRobotSpace()[0] : 0;
  }

  public double get3dTz() {
    return hasTarget() ? getTargetPoseRobotSpace()[2] : 0;
  }

  public double get3dYaw() {
    return hasTarget() ? getTargetPoseRobotSpace()[4] : 0;
  }

  public boolean isHubTag() {
    if (!hasTarget()) {
      return false;
    }
    int id = (int) getTid();
    return (id >= 2 && id <= 5) || (id >= 8 && id <= 11) 
            || (id >= 18 && id <= 21) || (id >= 24 && id <= 27);
           
  }

  public void setLedMode(int mode) {
    limelightTable.getEntry("ledMode").setNumber(mode);
  }

  public void setPipeline(int pipeline) {
    limelightTable.getEntry("pipeline").setNumber(pipeline);
  }

  public void setDisabled(boolean disabled) {
    this.disabled = disabled;
  }

  public Pose2d getBotPoseAsPose2d() {
    double[] poseArray = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
    if (!hasTarget() || poseArray.length < 7) {
      return null; 
    }
    return new Pose2d(
        poseArray[0], 
        poseArray[1], 
        Rotation2d.fromDegrees(poseArray[5])
    );
  }

  public double getTimestampSeconds() {
    double[] poseArray = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
    if (poseArray.length < 7) {
      return Timer.getFPGATimestamp();
    }
    double latencyMs = (poseArray.length >= 7) ? poseArray[6] : 0;
    return Timer.getFPGATimestamp() - (latencyMs / 1000.0);
  }
}