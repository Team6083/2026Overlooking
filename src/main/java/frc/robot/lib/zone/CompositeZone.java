// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.zone;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class CompositeZone implements Zone {
  private final Zone[] zones;

  public CompositeZone(Zone... zones) {
    this.zones = zones;
  }

  @Override
  public boolean contains(Translation2d translation) {
    for (Zone zone : zones) {
      if (zone.contains(translation)) {
        return true;
      }

    }
    return false;
  }
}