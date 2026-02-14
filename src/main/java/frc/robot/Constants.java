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

    public static final double intakeSpeed = 0.5;
    public static final double reverseIntakeSpeed = -0.5;

    public static final int pivotLeftId = 31;
    public static final int pivotRightId = 30;

    public static final int pivotEncoderId = 4;
    public static final double pivotFullRange = 360;
    public static final double pivotExpectedZero = 0;

    public static final double pivotDeployStopPosition = 100;
    public static final double pivotRetractPosition = 0;

    public static final double pivotSpeed = 0.2;
    public static final double reversePivotSpeed = -0.2;
  }

  public static final class ShooterConstants {
    public static final int shooterMotorID = 32;
    public static final double shooterMotorSpeed = 0.6;
    public static final double feedforwardKs = 0.2;
    public static final double feedforwardKv = 0.0028;
    public static final double feedforwardKa = 0;
    public static final int encoderChannelA = 0;
    public static final int encoderChannelB = 0;
    public static final double targetVelocity = 4000;
  }

  public static class TransportConstants {
    public static final int transportMotorID = 33;
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

  public static class ClimberConstants {
    public static final int motorId = 0;
    public static final double kP = 0.01;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final int currentLimit = 80;
    public static final double L1position = 25;
    public static final double L2position = 50;
    public static final int encoderChannel = 1;

  }
}
