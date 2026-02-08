// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  private final DriveSubsystem driveSubsystem;
  private final CommandXboxController mainController;
  private double magnification;

  public RobotContainer() {
    driveSubsystem = new DriveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    mainController = new CommandXboxController(0);
    configureBindings();
  }

  private double getMagnification() {
    magnification = mainController.leftBumper().getAsBoolean() ? 0.5 : 0.3;
    return magnification;
  }

  private void configureBindings() {
    driveSubsystem.setDefaultCommand(
        driveSubsystem.driveCommand(
            () -> MathUtil.applyDeadband(-mainController.getLeftY(), 0.1) * getMagnification(),
            () -> MathUtil.applyDeadband(-mainController.getLeftX(), 0.1) * getMagnification(),
            () -> MathUtil.applyDeadband(-mainController.getRightX(), 0.1) * getMagnification()));
    mainController.button(8).whileTrue(driveSubsystem.centerModulesCmd());
    mainController.button(7).whileTrue(driveSubsystem.zeroGyroCmd());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
