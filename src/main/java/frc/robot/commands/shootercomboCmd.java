// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class shootercomboCmd extends SequentialCommandGroup {
  /** Creates a new shootercomboCmd. */
  public shootercomboCmd(
    SwerveDrive drive,
    ShooterSubsystem shooter,
    TransportSubsystem transport,
    TagTracking vision
  ) {
    addCommands(
      Commands.parallel(
        new AutoAlignCmd(vision, drive),
        shooter.run(shooter::shoot)
      ).until(shooter::isShooterAtSpeed),
      transport.run(transport::transportIn)
    );
    addRequirements(drive, shooter, transport);
  }
}
