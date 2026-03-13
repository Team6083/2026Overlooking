// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.zone;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class RectZone implements Zone {
  private final double xMin;
  private final double xMax;
  private final double yMin;
  private final double yMax;

  public RectZone(double xMin, double xMax, double yMin, double yMax) {
    this.xMin = xMin;
    this.xMax = xMax;
    this.yMin = yMin;
    this.yMax = yMax;
  }

  @Override
  public boolean contains(Translation2d translation) {
    return translation.getX() >= xMin && translation.getX() <= xMax
        && translation.getY() >= yMin && translation.getY() <= yMax;
  }
}
