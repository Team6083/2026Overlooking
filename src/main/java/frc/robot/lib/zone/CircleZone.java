// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.zone;

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
  public boolean contains(Translation2d translation) {
    return translation.getDistance(center) <= radius;
  }
}
