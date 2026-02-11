// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveYagsl;
import frc.robot.lib.TagTracking;
import java.io.File;


public class RobotContainer {
  private final TagTracking shooterTracker;
  private final SwerveYagsl driveSubsystem;
  private final CommandXboxController mainController;
  private double magnification;

  public RobotContainer() {
    shooterTracker = new TagTracking("limelight-shooter");
    driveSubsystem = new SwerveYagsl(new File(Filesystem.getDeployDirectory(), "swerve"));
    mainController = new CommandXboxController(0);
    configureBindings();
  }

  private double getMagnification() {
    magnification = mainController.leftBumper().getAsBoolean() ? 0.6 : 0.3;
    return magnification;
  }

  private void configureBindings() {
    driveSubsystem.setDefaultCommand(
        driveSubsystem.driveCommand(
            () -> (MathUtil.applyDeadband(mainController.getLeftY(), 0.1) * getMagnification()),
            () -> (MathUtil.applyDeadband(mainController.getLeftX(), 0.1) * getMagnification()),
            () -> (MathUtil.applyDeadband(mainController.getRightX(), 0.1) * getMagnification()),
            true));
    mainController.button(8).onTrue(driveSubsystem.zeroGyroCommand());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
