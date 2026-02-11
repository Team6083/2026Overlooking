// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {
  public static final class IntakeConstants {
    public static final int intakeMotorId = 10;

    public static final double intakeSpeed = 0.5;
    public static final double reverseIntakeSpeed = -0.5;

    public static final int pivotLeftId = 11;
    public static final int pivotRightId = 12;

    public static final int pivotEncoderId = 4;
    public static final double pivotFullRange = 360;
    public static final double pivotExpectedZero = 0;

    public static final double pivotSpeed = 0.2;
    public static final double reversePivotSpeed = -0.2;
  }

  public static final class ShooterConstants {
    public static final int shooterMotorID = 0;
    public static final double shooterMotorSpeed = 0.6;
    public static final double feedforwardKs = 0;
    public static final double feedforwardKv = 0;
    public static final double feedforwardKa = 0;
    public static final int encoderChannelA = 0;
    public static final int encoderChannelB = 0;
    public static final double targetVelocity = 4000;
  }

  public static class TransportConstants {
    public static final int transportMotorID = 0;
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
