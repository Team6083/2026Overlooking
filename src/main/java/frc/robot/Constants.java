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

  public static final class IntakeConstants {
    public static final int intakeMotorId = 34;
    public static final int pivotLeftId = 30;
    public static final int pivotRightId = 31;
    public static final int pivotLeftEncoderId = 2;
    public static final int pivotRightEncoderId = 3;
    
    public static final double pivotLeftExpectedZero = 167;
    public static final double pivotRightExpectedZero = 141;

    public static final double pivotEncoderFullRange = 360;
    public static final double pivotDeployStopPosition = 107;
    public static final double pivotRetractStopPosition = 8;

    public static final double intakeSpeed = 0.6;
    public static final double reverseIntakeSpeed = -0.6;

    public static final double pivotSpeed = -0.6;
    public static final double reversePivotSpeed = 0.6;

    public static final double pivotFollowKp = 0.25;
    public static final double pivotFollowKi = 0;
    public static final double pivotFollowKd = 0;
    
    public static final boolean motorLeftInverted = false;
    public static final boolean motorRightInverted = true;
    public static final boolean encoderLeftInverted = false;
    public static final boolean encoderRightInverted = true;
    public static final double pivotFollowMinInput = 0;
    public static final double pivotFollowMaxInput = 360;
  }

  public static final class ShooterConstants {
    public static final int shooterMotorID = 33;
    public static final double shooterMotorSpeed = 0.6;
    public static final double feedforwardKs = 0.2;
    public static final double feedforwardKv = 0.0028;
    public static final double feedforwardKa = 0;
    public static final int encoderChannelA = 0;
    public static final int encoderChannelB = 1;
    public static final double targetVelocity = 4000;
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
}
