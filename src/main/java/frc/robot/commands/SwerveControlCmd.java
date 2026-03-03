
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveControlCmd extends Command {
  private final RobotContainer robotContainer;
  private final SwerveDrive swerveDrive;
  private final CommandXboxController mainController;
  private final SlewRateLimiter limiterX;
  private final SlewRateLimiter limiterY;
  private final SlewRateLimiter rotLimiter;
  private Supplier<Double> magnification;
  private Supplier<Double> rotMagnification;
  private double speedX;
  private double speedY;
  private double rotSpeed;

  /** Creates a new SwerveControlCmd. */
  public SwerveControlCmd(SwerveDrive swerveDrive, CommandXboxController mainController, Supplier<Double> magnification,
      Supplier<Double> rotMagnification) {
    this.swerveDrive = swerveDrive;
    this.mainController = mainController;
    this.limiterX = new SlewRateLimiter(3);
    this.limiterY = new SlewRateLimiter(3);
    this.rotLimiter = new SlewRateLimiter(3);
    this.magnification = magnification;
    this.rotMagnification = rotMagnification;
    addRequirements(swerveDrive);
    robotContainer = new RobotContainer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    speedX = -limiterX.calculate(MathUtil.applyDeadband(mainController.getLeftY(), 0.1)) * 4
        * magnification.get();
    speedY = -limiterY.calculate(MathUtil.applyDeadband(mainController.getLeftX(), 0.1)) * 4
        * magnification.get();
    rotSpeed = -rotLimiter.calculate(MathUtil.applyDeadband(mainController.getRightX(), 0.1))
        * 4 * rotMagnification.get();
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
