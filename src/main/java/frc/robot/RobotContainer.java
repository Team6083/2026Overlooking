// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ClimberSubsystem;
public class RobotContainer {
  private final ClimberSubsystem climber = new ClimberSubsystem();
  CommandXboxController mainController = new CommandXboxController(0);
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    mainController.a()
        .onTrue(climber.climbUpCmd());
    mainController.y()
        .onTrue(climber.climbDownCmd());
    mainController.x()
        .onTrue(climber.toLowRungCmd());
    mainController.b()
        .onTrue(climber.toMidRungCmd());
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}