// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberCmd extends Command {
  /** Creates a new climberCMD. */
  public enum ClimberAction {
    L1, L2, UP, DOWN
  }

  ClimberSubsystem climberSubsystem;
  ClimberAction action;

  public ClimberCmd(ClimberSubsystem climberSubsystem, ClimberAction action) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climberSubsystem = climberSubsystem;
    this.action = action;
    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (action) {
      case L1:
        climberSubsystem.toLowRung();
        break;
      case L2:
        climberSubsystem.toMidRung();
        break;
      case UP:
        climberSubsystem.climbUp();
        break;
      case DOWN:
        climberSubsystem.climbDown();
        break;
      default:
        climberSubsystem.stopClimb();
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberSubsystem.stopClimb();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
