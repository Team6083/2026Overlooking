// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrsSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoDrsCmd extends Command {
  /** Creates a new AutoLimelightPivotCmd. */
  private SwerveDrive swerveDrive;
  private DrsSubsystem drsSubsystem;

  public AutoDrsCmd(SwerveDrive swerveDrive, DrsSubsystem drsSubsystem) {
    this.swerveDrive = swerveDrive;
    this.drsSubsystem = drsSubsystem;
    addRequirements(drsSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isUnderRedLeftTrench() || isUnderRedRightTrench() || isUnderBlueLeftTrench() || isUnderBlueRightTrench()) {
      drsSubsystem.downDrs();
    } else {
      drsSubsystem.upDrs();
    }
  }

  public Boolean isUnderBlueLeftTrench() {

    double TrenchMinX = 3.9;
    double TrenchMaxX = 5.4;
    double TrenchMinY = 6.5;
    double TrenchMaxY = 8.0;

    if (swerveDrive.getPose2d().getX() > TrenchMinX && swerveDrive.getPose2d().getX() < TrenchMaxX
        && swerveDrive.getPose2d().getY() > TrenchMinY && swerveDrive.getPose2d().getY() < TrenchMaxY) {
      return true;
    }
    return false;
  }

  public Boolean isUnderBlueRightTrench() {

    double TrenchMinX = 3.9;
    double TrenchMaxX = 5.4;
    double TrenchMinY = 0;
    double TrenchMaxY = 1.5;

    if (swerveDrive.getPose2d().getX() > TrenchMinX && swerveDrive.getPose2d().getX() < TrenchMaxX
        && swerveDrive.getPose2d().getY() > TrenchMinY && swerveDrive.getPose2d().getY() < TrenchMaxY) {
      return true;
    }
    return false;
  }

  public Boolean isUnderRedLeftTrench() {

    double TrenchMinX = 11.2;
    double TrenchMaxX = 12.7;
    double TrenchMinY = 0;
    double TrenchMaxY = 1.5;

    if (swerveDrive.getPose2d().getX() > TrenchMinX && swerveDrive.getPose2d().getX() < TrenchMaxX
        && swerveDrive.getPose2d().getY() > TrenchMinY && swerveDrive.getPose2d().getY() < TrenchMaxY) {
      return true;
    }
    return false;
  }

  public Boolean isUnderRedRightTrench() {

    double TrenchMinX = 11.4;
    double TrenchMaxX = 12.7;
    double TrenchMinY = 6.5;
    double TrenchMaxY = 8;

    if (swerveDrive.getPose2d().getX() > TrenchMinX && swerveDrive.getPose2d().getX() < TrenchMaxX
        && swerveDrive.getPose2d().getY() > TrenchMinY && swerveDrive.getPose2d().getY() < TrenchMaxY) {
      return true;
    }
    return false;
  }
}
