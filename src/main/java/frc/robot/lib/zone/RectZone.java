// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.zone;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class RectZone implements Zone {
  private final double minX;
  private final double maxX;
  private final double minY;
  private final double maxY;

  public RectZone(double minX, double maxX, double minY, double maxY) {
    this.minX = minX;
    this.maxX = maxX;
    this.minY = minY;
    this.maxY = maxY;
  }

  @Override
  public boolean contains(Translation2d translation) {
    return translation.getX() >= minX && translation.getX() <= maxX
        && translation.getY() >= minY && translation.getY() <= maxY;
  }
}
