// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    Translation2d robotTranslation = new Translation2d(
        swerveDrive.getRobotRelativeSpeeds().vxMetersPerSecond,
        swerveDrive.getRobotRelativeSpeeds().vyMetersPerSecond);

    Translation2d robotToHubTranslation = new Translation2d(
        getHubPosition()[0] - swerveDrive.getPose2d().getX(),
        getHubPosition()[1] - swerveDrive.getPose2d().getY());

    Translation2d effectiveHubTranslation = robotToHubTranslation.plus(robotTranslation.times(0.5));

    double targetAngle = effectiveHubTranslation.getAngle().getDegrees() + 180;

    return yawPID.calculate(swerveDrive.getPose2d().getRotation().getDegrees(), targetAngle);
    // robotTranslation 是機器人速度 邊走邊射會需要加上他
    // robotToHubTranslation 是機器人到 hub 的向量

    // 下次：算一下方向對不對
  }

  public boolean isAlignedToHub() {
    return yawPID.atSetpoint();
  }

  private double[] getHubPosition() {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      return new double[] { FieldConstants.redHubX, FieldConstants.redHubY };
    }
    return new double[] { FieldConstants.blueHubX, FieldConstants.blueHubY };
  }
}