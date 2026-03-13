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

  public static final DriveBaseConstant TEST_CONFIG = new DriveBaseConstant(
      new SwerveModuleConstant(
          21, 26, 13, -0.164062, true, true, "FrontLeft"),
      new SwerveModuleConstant(
          25, 27, 11, -0.016602, true, true, "FrontRight"),
      new SwerveModuleConstant(
          22, 18, 14, -0.111328, true, true, "BackLeft"),
      new SwerveModuleConstant(
          23, 24, 12, 0.260986, true, true, "BackRight"));

  public static final DriveBaseConstant COMPETITION_CONFIG = new DriveBaseConstant(
      new SwerveModuleConstant(
          25, 27, 11, 0.337891, true, true, "FrontLeft"),
      new SwerveModuleConstant(
          26, 28, 13, 0.495361, true, true, "FrontRight"),
      new SwerveModuleConstant(
          24, 23, 12, -0.446289, true, true, "BackLeft"),
      new SwerveModuleConstant(
          22, 20, 14, 0.302979, true, true, "BackRight"));

  public static final class ModuleConstant {
    // define the radius of the wheel in meters
    public static final Distance kWheelRadius = Inches.of(2);
  }

  public static final class ClimberConstants {
    public static final int climbMotorId = 40;
    public static final boolean climbMotorInverted = true;

    public static final double climbUpSpeed = 0.4;
    public static final double climbDownSpeed = -0.4;

    public static final int climbEncoderIdA = 4;
    public static final int climbEncoderIdB = 5;
  }

  public static final class SwerveControlConstants {
    public static final double kFastMagnification = 0.6;
    public static final double kSlowMagnification = 0.3;
    public static final double kFastRotMagnification = 0.8;
    public static final double kSlowRotMagnification = 0.4;
  }

  public static final class IntakeConstants {
    public static final int intakeMotorId = 34;
    public static final int pivotLeftId = 30;
    public static final int pivotRightId = 31;
    public static final int pivotLeftEncoderId = 3;
    public static final int pivotRightEncoderId = 2;

    public static final double pivotLeftExpectedZero = -33;
    public static final double pivotRightExpectedZero = 48;

    public static final double pivotEncoderFullRange = 360;
    public static final double pivotDeployStopPosition = 95;
    public static final double pivotRetractStopPosition = 3;

    public static final boolean intakeInverted = true;

    public static final double intakeSpeed = 0.4;
    public static final double reverseIntakeSpeed = -0.4;

    public static final double deployPivotSpeed = 0.8;
    public static final double retractPivotSpeed = -1;

    public static final double pivotManualSpeed = 0.4;

    public static final double pivotFollowKp = 0.03;
    public static final double pivotFollowKi = 0;
    public static final double pivotFollowKd = 0;

    public static final boolean motorLeftInverted = false;
    public static final boolean motorRightInverted = true;
    public static final boolean encoderLeftInverted = true;
    public static final boolean encoderRightInverted = false;
  }

  public static final class ShooterConstants {
    public static final int shooterMotorID = 35;
    public static final double shooterMotorSpeed = 0.6;

    public static final int encoderChannelA = 0;
    public static final int encoderChannelB = 1;

    public static final double defaultTargetVelocity = 3000;
    // Tunable ball launch speed (m/s)
    public static final double ballSpeed = 2.0;

    public static final double maxShooterVelocity = 5700;

    public static final double feedforwardKs = 0.01;
    public static final double feedforwardKv = 0.00207;
    public static final double feedforwardKa = 0;

    public static final double shooterDistanceMultiplier = 2207.31;
    public static final double shooterDistanceExponent = 0.0017;
  }

  public static final class FeederConstants {
    public static final int feederMotorID = 32;
    public static final double feederMotorIn = 0.5;
    public static final double feederMotorOut = -0.5;
    public static final boolean feederMotorInverted = false;
  }

  public static final class TransportConstants {
    public static final int transportMotorLowerID = 36;
    public static final double transportMotorIn = 0.5;
    public static final double transportMotorOut = -0.5;
    public static final boolean transportMotorInverted = true;
  }

  public static class AutoConstants {
    public static final double kpTranslation = 5.75;
    public static final double kiTranslation = 0.0;
    public static final double kdTranslation = 0.75;
    public static final double kpRotation = 4.5;
    public static final double kiRotation = 0.0;
    public static final double kdRotation = 0.5;
  }

  public static class DrsConstants {
    public static final int servoMotorChannel = 4;
    public static final double upPosition = 1;
    public static final double downPosition = 0.25;
  }

  public static class FieldConstants {
    public static final double blueHubX = 4.611624;
    public static final double blueHubY = 4.021328;
    public static final double redHubX = 11.901424;
    public static final double redHubY = 4.021328;

    public static final double blueTrenchMinX = 3.65;
    public static final double blueTrenchMaxX = 5.65;
    public static final double redTrenchMinX = 10.95;
    public static final double redTrenchMaxX = 12.95;

    public static final double blueLeftTrenchMinY = 6.25;
    public static final double blueLeftTrenchMaxY = 8;
    public static final double blueRightTrenchMinY = 0;
    public static final double blueRightTrenchMaxY = 1.75;
  }
}
