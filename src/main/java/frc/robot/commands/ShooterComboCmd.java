// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterComboCmd extends ParallelCommandGroup {
  /** Creates a new shootercomboCmd. */
  public ShooterComboCmd(
      ShooterSubsystem shooterSubsystem,
      TransportSubsystem transportSubsystem,
      FeederSubsystem feederSubsystem) {
    addCommands(
        shooterSubsystem.shootCmd(),
        Commands.idle().until(shooterSubsystem::isShooterAtSpeed)
            .andThen(transportSubsystem.transportInCmd().alongWith(feederSubsystem.feedInCmd())));

    addRequirements(shooterSubsystem, transportSubsystem, feederSubsystem);
  }
}
