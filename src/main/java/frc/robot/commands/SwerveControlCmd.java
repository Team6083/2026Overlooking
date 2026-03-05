
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.lib.TagTracking;
import frc.robot.subsystems.swervedrive.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveControlCmd extends Command {
  private final SwerveDrive swerveDrive;
  private final CommandXboxController mainController;
  private final TagTracking vision;
  private final SlewRateLimiter limiterX;
  private final SlewRateLimiter limiterY;
  private final SlewRateLimiter rotLimiter;
  private final PIDController yawPID;
  private final Debouncer targetDebouncer;

  private double speedX;
  private double speedY;
  private double rotSpeed;
  private boolean isAligning;

  /** Creates a new SwerveControlCmd. */
  public SwerveControlCmd(SwerveDrive swerveDrive, CommandXboxController mainController, TagTracking vision) {
    this.swerveDrive = swerveDrive;
    this.mainController = mainController;
    this.vision = vision;
    this.limiterX = new SlewRateLimiter(4);
    this.limiterY = new SlewRateLimiter(4);
    this.rotLimiter = new SlewRateLimiter(5);
    this.yawPID = new PIDController(0.08, 0, 0);
    yawPID.setTolerance(1.0);
    this.targetDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kFalling);
    addRequirements(swerveDrive);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    yawPID.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speedX = calcSpeedX();
    speedY = calcSpeedY();
    rotSpeed = calcRotSpeed();
    swerveDrive.drive(speedX, speedY, rotSpeed, true);
  }

  private double getMagnification() {
    return mainController.leftBumper().getAsBoolean() ? 0.6 : 0.3;
  }

  private double getRotMagnification() {
    return mainController.leftBumper().getAsBoolean() ? 0.8 : 0.4;
  }

  private double calcSpeedX() {
    return -limiterX.calculate(MathUtil.applyDeadband(mainController.getLeftY(), 0.1)) * 4 * getMagnification();
  }

  private double calcSpeedY() {
    return -limiterY.calculate(MathUtil.applyDeadband(mainController.getLeftX(), 0.1)) * 4 * getMagnification();
  }

  private double calcRotSpeed() {
    boolean targetValid = targetDebouncer.calculate(vision.hasTarget() && vision.isHubTag());
    isAligning = mainController.rightBumper().getAsBoolean() && targetValid;

    if (isAligning) {
      double ballSpeed = 10.0;
      ChassisSpeeds DriveSpeeds = swerveDrive.getRobotRelativeSpeeds();
      double effectiveBallSpeed = ballSpeed + DriveSpeeds.vxMetersPerSecond;
      double compensation = Math.toDegrees(Math.atan2(DriveSpeeds.vyMetersPerSecond, effectiveBallSpeed));

      SmartDashboard.putNumber("compensation", compensation);
    
      return MathUtil.clamp(yawPID.calculate(vision.getTx(), compensation), -1.5, 1.5);
    } else {
      return -rotLimiter.calculate(MathUtil.applyDeadband(mainController.getRightX(), 0.1)) * 4 * getRotMagnification();
    }
  }

  public boolean isAlignedToHub() {
    return isAligning && yawPID.atSetpoint();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    yawPID.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
