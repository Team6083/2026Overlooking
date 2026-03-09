// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Add your docs here. */
public interface SwerveDrive extends Subsystem {
  void addVisionMeasurement(Pose2d visionRobotPose, double timestamp);

  void addVisionMeasurement(Pose2d visionRobotPose, double timestamp, Vector<N3> visionStdDevs);

  void drive(double translationX, double translationY, double angularRotationX, boolean fieldRelative);

  void drive(ChassisSpeeds speeds);

  void zeroGyro();

  Command zeroGyroCommand();

  Pose2d getPose2d();

  void resetPose(Pose2d pose);

  ChassisSpeeds getRobotRelativeSpeeds();

  Command sysIdQuasistaticFCmd();

  Command sysIdDynamicFCmd();

  Command sysIdQuasistaticRCmd();

  Command sysIdDynamicRCmd();

  Rotation2d getGyroRotation2d();
}