// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {
  public static final class IntakeConstants {
    public static final int intakeMotorId = 34;

    public static final double intakeSpeed = 0.5;
    public static final double reverseIntakeSpeed = -0.5;

    public static final int pivotLeftId = 31;
    public static final int pivotRightId = 30;

    public static final int pivotLeftEncoderId = 2;
    public static final int pivotRightEncoderId = 3;
    public static final double pivotLeftExpectedZero = 214;
    public static final double pivotRightExpectedZero = 340;

    public static final double pivotEncoderFullRange = 360;

    public static final double pivotDeployStopPosition = 103;
    public static final double pivotRetractPosition = 3;

    public static final double pivotSpeed = 0.4;
    public static final double reversePivotSpeed = -0.6;
  }

  public static final class ShooterConstants {
    public static final int shooterMotorID = 32;
    public static final double shooterMotorSpeed = 0.6;
    public static final double feedforwardKs = 0.2;
    public static final double feedforwardKv = 0.0028;
    public static final double feedforwardKa = 0;
    public static final int encoderChannelA = 0;
    public static final int encoderChannelB = 1;
    public static final double targetVelocity = 4000;
  }

  public static class TransportConstants {
    public static final int transportMotorID = 33;
    public static final double transportMotorIn = 0.2;
    public static final double transportMotorOut = 0.2;
  }
}
