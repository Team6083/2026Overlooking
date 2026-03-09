// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class Constants {
  public record SwerveModuleConstant(
      int turningMotorId,
      int driveMotorId,
      int canCoderId,
      double canCoderOffset,
      boolean turningInverted,
      boolean driveInverted,
      String name) {
  }

  public record DriveBaseConstant(
      SwerveModuleConstant frontLeft,
      SwerveModuleConstant frontRight,
      SwerveModuleConstant backLeft,
      SwerveModuleConstant backRight) {
  }

  public static final DriveBaseConstant COMPETITION_CONFIG = new DriveBaseConstant(
      new SwerveModuleConstant(
          21, 26, 13, -0.164062, true, true, "FrontLeft"),
      new SwerveModuleConstant(
          25, 27, 11, -0.016602, true, true, "FrontRight"),
      new SwerveModuleConstant(
          22, 18, 14, -0.111328, true, true, "BackLeft"),
      new SwerveModuleConstant(
          23, 24, 12, 0.260986, true, true, "BackRight"));

  public static final DriveBaseConstant CHASSIS_CONFIG = new DriveBaseConstant(
      new SwerveModuleConstant(
          21, 26, 13, -0.164062, true, true, "FrontLeft"),
      new SwerveModuleConstant(
          25, 27, 11, -0.016602, true, true, "FrontRight"),
      new SwerveModuleConstant(
          22, 18, 14, -0.254395, true, true, "BackLeft"),
      new SwerveModuleConstant(
          23, 24, 12, 0.260986, true, true, "BackRight"));

  public static final class ModuleConstant {
    // define the radius of the wheel in meters
    public static final Distance kWheelRadius = Inches.of(2);
  }

  public static final class IntakeConstants {
    public static final int intakeMotorId = 34;

    public static final boolean intakeInverted = true;

    public static final double intakeSpeed = 0.6;

    public static final double reverseIntakeSpeed = -0.6;

    public static final int pivotLeftId = 31;
    public static final int pivotRightId = 30;

    public static final int pivotEncoderId = 2;
    public static final double pivotFullRange = 360;
    public static final double pivotExpectedZero = 152.5;

    public static final double pivotDeployStopPosition = 100;
    public static final double pivotRetractPosition = 0;

    public static final double pivotSpeed = 0.4;
    public static final double reversePivotSpeed = -0.8;
  }

  public static final class ShooterConstants {
    public static final int shooterMotorID = 35;
    public static final double shooterMotorSpeed = 0.6;
    public static final double feedforwardKs = 0.2;
    public static final double feedforwardKv = 0.002;
    public static final double feedforwardKa = 0;
    public static final int encoderChannelA = 0;
    public static final int encoderChannelB = 1;
    public static final double targetVelocity = 5700;
  }

  public static class TransportConstants {
    public static final int transportMotorID = 32;
    public static final double transportMotorIn = 0.2;
    public static final double transportMotorOut = 0.2;
  }

  public static class AutoConstants {
    public static final double kpTranslation = 5.75;
    public static final double kiTranslation = 0.0;
    public static final double kdTranslation = 0.75;
    public static final double kpRotation = 4.5;
    public static final double kiRotation = 0.0;
    public static final double kdRotation = 0.5;
  }

  public static class LimelightPivotConstants {
    public static final int servoMotorChannel = 4;
    public static final double upPosition = 0.25;
    public static final double downPosition = 1;
  }
}
