// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.lib.TagTracking;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CalculateSpeedShooterCmd extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final TagTracking tagTracking;
  private final frc.robot.subsystems.swervedrive.SwerveDrive swerveDrive;

  private final Debouncer targetDebouncer = new Debouncer(0.2);

  /** Creates a new CalculateSpeedShooterCmd. */
  public CalculateSpeedShooterCmd(ShooterSubsystem shooterSubsystem, TagTracking tagTracking,
      frc.robot.subsystems.swervedrive.SwerveDrive swerveDrive) {
    this.shooterSubsystem = shooterSubsystem;
    this.tagTracking = tagTracking;
    this.swerveDrive = swerveDrive;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean isTargetValid = targetDebouncer.calculate(tagTracking.hasTarget() && tagTracking.isHubTag());
    double targetVelocity;
    double distanceX;
    double distanceY;
    Distance distance;

    distanceX = swerveDrive.getPose2d().getX() - getHubPositionX()-59.69;
    distanceY = swerveDrive.getPose2d().getY() - getHubPositionY()-59.69;

    if (isTargetValid) {
      distance = Meters.of(Math.sqrt(Math.pow(distanceX, 2) + Math.pow(distanceY, 2)));
      targetVelocity = 2570.3 * Math.exp(0.00436 * distance.in(Centimeters)); // 2570.2e^0.0044x
    } else {
      distance = Meters.of(-1);
      targetVelocity = ShooterConstants.targetVelocity;
    }

    shooterSubsystem.shoot(targetVelocity);
  }

  private double getHubPositionX() {
    if (DriverStation.getAlliance().isPresent() &&
        DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      return FieldConstants.redHubX;
    }
    return FieldConstants.blueHubX;
  }

  private double getHubPositionY() {
    if (DriverStation.getAlliance().isPresent() &&
        DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      return FieldConstants.redHubY;
    }
    return FieldConstants.blueHubY;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShooter();
  }
}
