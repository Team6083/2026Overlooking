// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class Constants {
  public static final class DriveBaseConstant {
    // turningMotor ID
    public static final int leftFrontTurningMotorId = 23;

    // driveMotor ID
    public static final int leftFrontDriveMotorId = 24;
  }

  public static final class ModuleConstant {
    // define the radius of the wheel in meters
    public static final Distance kWheelRadius = Inches.of(2);
  }
}
