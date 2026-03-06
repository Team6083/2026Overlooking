// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.lib.TagTracking;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CalculateSpeedShooterCmd extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final frc.robot.subsystems.swervedrive.SwerveDrive swerveDrive;

  private double targetVelocity;

  /** Creates a new CalculateSpeedShooterCmd. */
  public CalculateSpeedShooterCmd(ShooterSubsystem shooterSubsystem, TagTracking tagTracking,
      frc.robot.subsystems.swervedrive.SwerveDrive swerveDrive) {
    this.shooterSubsystem = shooterSubsystem;
    this.swerveDrive = swerveDrive;
    addRequirements(shooterSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Distance distanceX;
    Distance distanceY;
    Distance distance;

    distanceX = Meters.of(swerveDrive.getPose2d().getX() - getHubPositionX());
    distanceY = Meters.of(swerveDrive.getPose2d().getY() - getHubPositionY());

    distance = Meters.of(Math.sqrt(Math.pow(distanceX.in(Meters), 2) + Math.pow(distanceY.in(Meters), 2)));
    targetVelocity = MathUtil.clamp(ShooterConstants.shooterDistanceMultiplier
        * Math.exp(ShooterConstants.shooterDistanceExponent * distance.in(Centimeters)),
        0.0, ShooterConstants.maxShooterVelocity); // 1981.4e^0.00436x

    shooterSubsystem.shoot(targetVelocity);

    SmartDashboard.putNumber("shooterCalculatedVelocity", targetVelocity);
    SmartDashboard.putNumber("shooterDistance", distance.in(Centimeters));
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

  private double getTargetVelocity() {
    return this.targetVelocity;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShooter();
  }
}
