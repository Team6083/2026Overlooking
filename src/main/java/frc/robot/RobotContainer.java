// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final CommandXboxController driverController = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    
    // 按住A鍵吸球放開停止
    driverController.a().whileTrue(
        new RunCommand(() -> intakeSubsystem.intake(), intakeSubsystem)
    ).onFalse(new InstantCommand(() -> intakeSubsystem.stopIntake(), intakeSubsystem));

    // 按住B鍵反轉放開停止
    driverController.b().whileTrue(
        new RunCommand(() -> intakeSubsystem.reverseIntake(), intakeSubsystem)
    ).onFalse(new InstantCommand(() -> intakeSubsystem.stopIntake(), intakeSubsystem));

    // ------------------- Pivot 角度控制 -------------------

    // 按住 Y 鍵抬起放開停止
    driverController.y().whileTrue(
        new RunCommand(() -> intakeSubsystem.rotateUp(), intakeSubsystem)
    ).onFalse(new InstantCommand(() -> intakeSubsystem.stopRotate(), intakeSubsystem));

    // 按住 X鍵放下放開停止
    driverController.x().whileTrue(
        new RunCommand(() -> intakeSubsystem.rotateDown(), intakeSubsystem)
    ).onFalse(new InstantCommand(() -> intakeSubsystem.stopRotate(), intakeSubsystem));
  }

  public Command getAutonomousCommand() {
    return null; 
  }
}
