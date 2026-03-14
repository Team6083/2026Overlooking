// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import java.util.function.Supplier;

public class AimAssistCmd extends SwerveControlCmd {
  private final PIDController yawPID;

  public AimAssistCmd(SwerveDrive swerveDrive, CommandXboxController mainController,
      Supplier<Boolean> shouldSprint, Supplier<Boolean> shouldLock) {
    super(swerveDrive, mainController, shouldSprint, shouldLock);
    this.yawPID = new PIDController(0.07, 0.002, 0);
    this.yawPID.setTolerance(0.5);
    this.yawPID.enableContinuousInput(-180, 180);
    this.yawPID.setIZone(3);
  }

  @Override
  public void initialize() {
    super.initialize();
    yawPID.reset();
  }

  @Override
  protected double calcRotSpeed() {
    Pose2d robotPose = swerveDrive.getPose2d();
    double[] hub = getHubPosition();
    double dx = hub[0] - robotPose.getX();
    double dy = hub[1] - robotPose.getY();
    double targetAngle = Math.toDegrees(Math.atan2(dy, dx)) + 180;
    final ChassisSpeeds driveSpeeds = swerveDrive.getRobotRelativeSpeeds();

    double currentAngle = robotPose.getRotation().getDegrees();
    double error = targetAngle - currentAngle;
    if (error > 180) {
      error -= 360;
    }
    if (error < -180) {
      error += 360;
    }
    double effectiveBallSpeed = ShooterConstants.ballSpeed + driveSpeeds.vxMetersPerSecond;
    double compensation = Math.toDegrees(Math.atan2(-driveSpeeds.vyMetersPerSecond, effectiveBallSpeed));
    double output = MathUtil.clamp(-(yawPID.calculate(error, compensation)), -1.5, 1.5);
    double actualOutput = isAlignedToHub() ? 0 : output;
    SmartDashboard.putNumber("AimAssist/compensation", compensation);
    SmartDashboard.putNumber("AimAssist/error", error);
    SmartDashboard.putNumber("AimAssistCmd/output", actualOutput);
    SmartDashboard.putData("AimAssistCmd/yawPID", yawPID);
    SmartDashboard.putNumber("AimAssistCmd/targetAngle", targetAngle);
    SmartDashboard.putNumber("AimAssistCmd/currentAngle", currentAngle);
    SmartDashboard.putNumber("AimAssistCmd/effectiveBallSpeed", effectiveBallSpeed);
    return actualOutput;
  }

  public boolean isAlignedToHub() {
    return yawPID.atSetpoint();
  }

  private double[] getHubPosition() {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      return new double[]{FieldConstants.redHubX, FieldConstants.redHubY};
    }
    return new double[]{FieldConstants.blueHubX, FieldConstants.blueHubY};
  }
}