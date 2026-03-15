
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ModuleConstant;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveControlCmd extends Command {
  protected final SwerveDrive swerveDrive;

  protected final CommandXboxController mainController;

  private final SlewRateLimiter limiterX;
  private final SlewRateLimiter limiterY;
  private final SlewRateLimiter rotLimiter;

  private Supplier<Boolean> shouldSprint;
  private Supplier<Boolean> shouldLock;

  /** Creates a new SwerveControlCmd. */
  public SwerveControlCmd(SwerveDrive swerveDrive, CommandXboxController mainController,
      Supplier<Boolean> shouldSprint, Supplier<Boolean> shouldLock) {
    this.swerveDrive = swerveDrive;
    this.mainController = mainController;

    this.limiterX = new SlewRateLimiter(4);
    this.limiterY = new SlewRateLimiter(4);
    this.rotLimiter = new SlewRateLimiter(5);

    this.shouldSprint = shouldSprint;
    this.shouldLock = shouldLock;
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedX = calcSpeedX();
    double speedY = calcSpeedY();
    double rotSpeed = calcRotSpeed();

    boolean isJoystickQuiet = Math.abs(speedX) < 0.1
        && Math.abs(speedY) < 0.1 && Math.abs(rotSpeed) < 0.1;

    if (shouldLock.get() && isJoystickQuiet) {
      swerveDrive.lockPose();
    } else {
      swerveDrive.drive(speedX, speedY, rotSpeed, true);
    }
  }

  private double getMagnification() {
    return shouldSprint.get() ? 0.6 : 0.3;
  }

  private double getRotMagnification() {
    return shouldSprint.get() ? 0.8 : 0.4;
  }

  protected double calcSpeedX() {
    return -limiterX.calculate(MathUtil.applyDeadband(mainController.getLeftY(), 0.1))
        * ModuleConstant.kMaxModuleSpeed.in(MetersPerSecond) * getMagnification();
  }

  protected double calcSpeedY() {
    return -limiterY.calculate(MathUtil.applyDeadband(mainController.getLeftX(), 0.1))
        * ModuleConstant.kMaxModuleSpeed.in(MetersPerSecond) * getMagnification();
  }

  protected double calcRotSpeed() {
    return -rotLimiter.calculate(MathUtil.applyDeadband(mainController.getRightX(), 0.1))
        * ModuleConstant.kMaxModuleSpeed.in(MetersPerSecond) * getRotMagnification();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
