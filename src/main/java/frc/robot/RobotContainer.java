// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.drivebase.SwerveDrive;

public class RobotContainer {
  private final SwerveDrive swerveDrive = new SwerveDrive();
  private final CommandXboxController driverController = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();

    swerveDrive.setDefaultCommand(
        new RunCommand(
        () -> swerveDrive.drive(
            -MathUtil.applyDeadband(driverController.getLeftY(), 0.1),
            -MathUtil.applyDeadband(driverController.getLeftX(), 0.1),
            -MathUtil.applyDeadband(driverController.getRightX(), 0.1)
        ),
        swerveDrive
        )
    );
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}