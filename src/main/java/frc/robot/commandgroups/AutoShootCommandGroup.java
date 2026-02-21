// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoAlignCmd;
import frc.robot.commands.ShooterComboCmd;
import frc.robot.lib.TagTracking;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShootCommandGroup extends SequentialCommandGroup {
  TagTracking tagTracking;
  SwerveDrive drive;
  ShooterSubsystem shooterSubsystem;
  TransportSubsystem transportSubsystem;

  Debouncer tagDebouncer = new Debouncer(1, DebounceType.kFalling);

  /** Creates a new TagTrackingWithShoot. */
  public AutoShootCommandGroup(TagTracking tagTracking, SwerveDrive swerveDrive, ShooterSubsystem shooterSubsystem,
      TransportSubsystem transportSubsystem) {
    this.tagTracking = tagTracking;
    this.drive = swerveDrive;
    this.shooterSubsystem = shooterSubsystem;
    this.transportSubsystem = transportSubsystem;

    addCommands(
        Commands.either(new SequentialCommandGroup(new AutoAlignCmd(tagTracking, swerveDrive),
            new ShooterComboCmd(shooterSubsystem, transportSubsystem)), Commands.none(),
            () -> tagDebouncer.calculate(tagTracking.isHubTag())));
  }
}
