// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrepShooterCmd extends ParallelCommandGroup {
  /** Creates a new PrepShooterCmd. */
  public PrepShooterCmd(
      SwerveDrive swerveDrive,
      ShooterSubsystem shooterSubsystem,
      TransportSubsystem transportSubsystem,
      FeederSubsystem feederSubsystem,
      IntakeSubsystem intakeSubsystem) {

    addCommands(
        new CalculateSpeedShooterCmd(shooterSubsystem, swerveDrive),
        Commands.idle().until(shooterSubsystem::isShooterAtSpeed)
            .andThen(transportSubsystem.transportInCmd()
                .alongWith(feederSubsystem.feedInCmd())
                .alongWith(intakeSubsystem.intakeCmd())));
  }
}
