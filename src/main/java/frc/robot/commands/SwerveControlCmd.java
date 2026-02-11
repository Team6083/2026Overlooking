// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swervedrive.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveControlCmd extends Command {
  private final SwerveDrive swerveDrive;
  private final CommandXboxController mainController;
  private final SlewRateLimiter limiterX;
  private final SlewRateLimiter limiterY;
  private final SlewRateLimiter rotLimiter;
  private double magnification;
  private double rotMagnification;
  private double speedX;
  private double speedY;
  private double rotSpeed;

  /** Creates a new SwerveControlCmd. */
  public SwerveControlCmd(SwerveDrive swerveDrive, CommandXboxController mainController) {
    this.swerveDrive = swerveDrive;
    this.mainController = mainController;
    this.limiterX = new SlewRateLimiter(3);
    this.limiterY = new SlewRateLimiter(3);
    this.rotLimiter = new SlewRateLimiter(3);
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    magnification = mainController.leftBumper().getAsBoolean() ? 0.6 : 0.3;
    rotMagnification = mainController.leftBumper().getAsBoolean() ? 0.8 : 0.4;

    speedX = -limiterX.calculate(MathUtil.applyDeadband(mainController.getLeftY(), 0.1)) * 4
        * magnification;
    speedY = -limiterY.calculate(MathUtil.applyDeadband(mainController.getLeftX(), 0.1)) * 4
        * magnification;
    rotSpeed = -rotLimiter.calculate(MathUtil.applyDeadband(mainController.getRightX(), 0.1))
        * 4 * rotMagnification;
    swerveDrive.drive(speedX, speedY, rotSpeed, true);
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
