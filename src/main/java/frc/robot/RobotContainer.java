// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveControlCmd;
import frc.robot.subsystems.swervedrive.YagslSwerve;

import java.io.File;

public class RobotContainer {
  private final YagslSwerve yagslSwerve;
  private final CommandXboxController mainController = new CommandXboxController(0);

  public RobotContainer() {
    yagslSwerve = new YagslSwerve(new File(Filesystem.getDeployDirectory(), "swerve"));
    configureBindings();
  }

  private void configureBindings() {
    yagslSwerve.setDefaultCommand(new SwerveControlCmd(yagslSwerve, mainController));
    mainController.button(8).onTrue(yagslSwerve.zeroGyroCommand());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}