// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.lib.TagTracking;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CalculateSpeedShooterCmd extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final TagTracking tagTracking;
  private final Debouncer targetDebouncer = new Debouncer(0.2);
  private static double distance;
  public static double targetVelocity;

  /** Creates a new CalculateSpeedShooterCmd. */
  public CalculateSpeedShooterCmd(ShooterSubsystem shooterSubsystem, TagTracking tagTracking) {
    this.shooterSubsystem = shooterSubsystem;
    this.tagTracking = tagTracking;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean isTargetValid = targetDebouncer.calculate(tagTracking.hasTarget() && tagTracking.isHubTag());
    if (isTargetValid) {
      distance = Math.sqrt(Math.pow(tagTracking.getTx(), 2) + Math.pow(tagTracking.getTy(), 2));
      double targetVelocity = 2588 * Math.exp(0.00431 * distance);
      CalculateSpeedShooterCmd.targetVelocity = targetVelocity;
      shooterSubsystem.shootCmd();
    } else {
      double targetVelocity = ShooterConstants.targetVelocity;
      CalculateSpeedShooterCmd.targetVelocity = targetVelocity;
      shooterSubsystem.shootCmd();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
