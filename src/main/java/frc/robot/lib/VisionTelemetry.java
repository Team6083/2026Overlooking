// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.Elastic.Notification;
import frc.robot.lib.Elastic.Notification.NotificationLevel;

public class VisionTelemetry {
  private final TagTracking vision;
  private boolean hadTarget = false;
  private boolean wasHubTag = false;

  public VisionTelemetry(TagTracking vision) {
    this.vision = vision;
  }

  public void update() {
    updateVisionDisplay();
  }

  private void updateVisionDisplay() {
    boolean hasTarget = vision.hasTarget();
    boolean isHub = vision.isHubTag();

    SmartDashboard.putBoolean("hasTarget", hasTarget);
    
    if (hasTarget) {
      SmartDashboard.putNumber("tagId", vision.getTid());
      SmartDashboard.putBoolean("isHubTag", isHub);
      SmartDashboard.putNumber("txDeg", vision.getTx());
      SmartDashboard.putNumber("tyDeg", vision.getTy());
      SmartDashboard.putNumber("txMeters", vision.get3dTx());
      SmartDashboard.putNumber("tzMeters", vision.get3dTz());
      SmartDashboard.putNumber("yawDeg", vision.get3dYaw());

      if (isHub && (!hadTarget || !wasHubTag)) {
        Elastic.sendNotification(
          new Notification(NotificationLevel.INFO, "Vision System", "HUB Locked")
            .withDisplaySeconds(2.0)
        );
      }
      
      wasHubTag = isHub;
    } else {
      SmartDashboard.putNumber("tagId", 0);
      SmartDashboard.putBoolean("isHubTag", false);
      SmartDashboard.putNumber("txDeg", 0);
      SmartDashboard.putNumber("tyDeg", 0);
      SmartDashboard.putNumber("txMeters", 0);
      SmartDashboard.putNumber("tzMeters", 0);
      SmartDashboard.putNumber("yawDeg", 0);
      
      wasHubTag = false;
    }
    
    hadTarget = hasTarget;
  }
}