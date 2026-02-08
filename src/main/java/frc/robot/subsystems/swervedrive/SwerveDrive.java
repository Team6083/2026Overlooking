// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;

/** Add your docs here. */
public interface SwerveDrive extends Subsystem {
  void drive(double translationX, double translationY, double angularRotationX, boolean fieldRelative);

  void zeroGyro();

  Command driveCommand(double translationX, double translationY, double angularRotationX, boolean fieldRelative);

  Command driveCommand(Supplier<Double> translationX, Supplier<Double> translationY,
      Supplier<Double> angularRotationX, boolean fieldRelative);

  Command zeroGyroCommand();

  Pose2d getPose2d();

  void resetPose(Pose2d pose);

  ChassisSpeeds getRobotRelativeSpeeds();
}
