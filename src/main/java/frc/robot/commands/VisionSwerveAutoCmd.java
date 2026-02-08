// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.Vision;
import frc.robot.subsystems.swervedrive.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class VisionSwerveAutoCmd extends Command {
  private Vision vision;
  private SwerveDrive swerveDrive;

  /** Creates a new VisionSwerveAutoCmd. */
  public VisionSwerveAutoCmd(SwerveDrive swerveDrive, Vision vision) {
    this.swerveDrive = swerveDrive;
    this.vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (vision.getVisionX() > 0.2) {
      swerveDrive.drive(0, 0, 0.5, false); // Move right
    } else if (vision.getVisionX() < -0.2) {
      swerveDrive.drive(0, 0, -0.5, false); // Move left
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
