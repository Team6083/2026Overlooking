// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoLimelightPivotCmd extends Command {
  /** Creates a new AutoLimelightPivotCmd. */
  private SwerveDrive swerveDrive;
  Boolean isUnderTrench = false;

  public AutoLimelightPivotCmd(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isUnderRedLeftTrench() || isUnderRedRightTrench() || isUnderBlueLeftTrench() || isUnderBlueRightTrench()) {
      isUnderTrench = true;
    } else {
      isUnderTrench = false;
    }

    SmartDashboard.putBoolean("isUnderTrench", isUnderTrench);
  }

  public Boolean isUnderBlueLeftTrench() {

    double TrenchMinX = 4;
    double TrenchMaxX = 5.22;
    double TrenchMinY = 6.8;
    double TrenchMaxY = 8.08;

    if (swerveDrive.getPose2d().getX() > TrenchMinX && swerveDrive.getPose2d().getX() < TrenchMaxX
        && swerveDrive.getPose2d().getY() > TrenchMinY && swerveDrive.getPose2d().getY() < TrenchMaxY) {
      return true;
    }
    return false;
  }

  public Boolean isUnderBlueRightTrench() {

    double TrenchMinX = 4;
    double TrenchMaxX = 5.22;
    double TrenchMinY = 0;
    double TrenchMaxY = 1.25;

    if (swerveDrive.getPose2d().getX() > TrenchMinX && swerveDrive.getPose2d().getX() < TrenchMaxX
        && swerveDrive.getPose2d().getY() > TrenchMinY && swerveDrive.getPose2d().getY() < TrenchMaxY) {
      return true;
    }
    return false;
  }

  public Boolean isUnderRedLeftTrench() {

    double TrenchMinX = 11.362;
    double TrenchMaxX = 12.553;
    double TrenchMinY = 0;
    double TrenchMaxY = 1.25;

    if (swerveDrive.getPose2d().getX() > TrenchMinX && swerveDrive.getPose2d().getX() < TrenchMaxX
        && swerveDrive.getPose2d().getY() > TrenchMinY && swerveDrive.getPose2d().getY() < TrenchMaxY) {
      return true;
    }
    return false;
  }

  public Boolean isUnderRedRightTrench() {

    double TrenchMinX = 11.362;
    double TrenchMaxX = 12.553;
    double TrenchMinY = 6.8;
    double TrenchMaxY = 8.08;

    if (swerveDrive.getPose2d().getX() > TrenchMinX && swerveDrive.getPose2d().getX() < TrenchMaxX
        && swerveDrive.getPose2d().getY() > TrenchMinY && swerveDrive.getPose2d().getY() < TrenchMaxY) {
      return true;
    }
    return false;
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
