// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.Zone;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class CircleZone implements Zone {
  private final Translation2d center;
  private final double radius;

  public CircleZone(Translation2d center, double radius) {
    this.center = center;
    this.radius = radius;
  }

  @Override
  public boolean contains(Pose2d pose) {
    double dx = pose.getX() - center.getX();
    double dy = pose.getY() - center.getY();
    return dx * dx + dy * dy <= radius * radius;
  }
}
