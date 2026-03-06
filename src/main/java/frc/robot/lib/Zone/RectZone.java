// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.Zone;

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
  public boolean contains(double x, double y) {
    return x >= xMin && x <= xMax && y >= yMin && y <= yMax;
  }

}
