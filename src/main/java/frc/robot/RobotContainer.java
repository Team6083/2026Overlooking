// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.driveBase.SwerveDrive;
import frc.robot.driveBase.SwerveModule;

public class RobotContainer {
  private final SwerveDrive swerveDrive = new SwerveDrive();
  private final SwerveModule m_testModule = swerveDrive.m_testModule;
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();

    m_testModule.setDefaultCommand(
        new RunCommand(() -> {
          double speed = -MathUtil.applyDeadband(m_driverController.getLeftY(), 0.1) * 2.0;

          Rotation2d angle = Rotation2d.fromDegrees(
                MathUtil.applyDeadband(m_driverController.getRightX(), 0.1) * 180
            );

          SwerveModuleState desiredState = new SwerveModuleState(speed, angle);
          m_testModule.setDesiredState(desiredState);
        }, m_testModule) 
    );
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}