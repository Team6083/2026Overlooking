
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveControlConstants;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveControlCmd extends Command {
  private final SwerveDrive swerveDrive;
  private final CommandXboxController mainController;
  private final SlewRateLimiter limiterX;
  private final SlewRateLimiter limiterY;
  private final SlewRateLimiter rotLimiter;
  private final PIDController yawPID;
  private boolean isAligning;
  private double speedX;
  private double speedY;
  private double rotSpeed;
  private Supplier<Boolean> shouldSprint;
  private ChassisSpeeds driveSpeeds;

  /** Creates a new SwerveControlCmd. */
  public SwerveControlCmd(SwerveDrive swerveDrive, CommandXboxController mainController,
      Supplier<Boolean> shouldSprint) {
    this.swerveDrive = swerveDrive;
    this.mainController = mainController;
    this.limiterX = new SlewRateLimiter(4);
    this.limiterY = new SlewRateLimiter(4);
    this.rotLimiter = new SlewRateLimiter(5);
    this.yawPID = new PIDController(0.04, 0, 0);
    yawPID.setTolerance(1.0);
    this.shouldSprint = shouldSprint;    
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
    Boolean isSprint = shouldSprint.get();
    swerveDrive.drive(speedX, speedY, rotSpeed, true);
  }

  private double[] getHubPosition() {
    if (DriverStation.getAlliance().isPresent() 
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      return new double[]{FieldConstants.redHubX, FieldConstants.redHubY}; 
    }
    return new double[]{FieldConstants.blueHubX, FieldConstants.blueHubY};
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
    isAligning = mainController.rightBumper().getAsBoolean();

    if (isAligning) {
      Pose2d robotPose = swerveDrive.getPose2d();
      double[] hub = getHubPosition();
      double dx = hub[0] - robotPose.getX();
      double dy = hub[1] - robotPose.getY();
      double targetAngle = Math.toDegrees(Math.atan2(dy, dx)) + 180;
      driveSpeeds = swerveDrive.getRobotRelativeSpeeds();

      double currentAngle = robotPose.getRotation().getDegrees();
      double error = targetAngle - currentAngle;
      if (error > 180) {
        error -= 360;
      }
      if (error < -180) {
        error += 360;
      }
      error = MathUtil.applyDeadband(error, 1.5);

      double effectiveBallSpeed = ShooterConstants.ballSpeed + driveSpeeds.vxMetersPerSecond;
      double compensation = Math.toDegrees(Math.atan2(driveSpeeds.vyMetersPerSecond, effectiveBallSpeed));

      SmartDashboard.putNumber("headingError", error);
      SmartDashboard.putNumber("compensation", compensation);
    
      return MathUtil.clamp(yawPID.calculate(error, compensation), -1.5, 1.5);
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
