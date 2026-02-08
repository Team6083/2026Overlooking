package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.TagTracking;
import frc.robot.subsystems.swervedrive.SwerveDrive;

public class AutoAlignCmd extends Command {

  private final TagTracking vision;
  private final SwerveDrive drive;
  private final PIDController yawPID;
  private final PIDController distPID; 
  private int lostTargetFrames = 0;

  public AutoAlignCmd(TagTracking vision, SwerveDrive drive) {
    this.vision = vision;
    this.drive = drive;
    this.yawPID = new PIDController(0.05, 0, 0);
    yawPID.setTolerance(1.5);
    this.distPID = new PIDController(0.05, 0, 0);
    distPID.setTolerance(0.15);
    addRequirements(drive);
  }
  
  @Override
  public void initialize() {
    yawPID.reset();
    distPID.reset();
  }

  @Override
  public void execute() {
    if (!vision.hasTarget() || !vision.isHubTag()) {
      lostTargetFrames++;
      drive.drive(0, 0, 0, false);
      return;
    }

    lostTargetFrames = 0;
    double yawOutput = yawPID.calculate(vision.get3dYaw(), 0);
    yawOutput = MathUtil.clamp(yawOutput, -0.5, 0.5);
    double distOutput = distPID.calculate(vision.get3dTz(), 1.2);
    distOutput = MathUtil.clamp(distOutput, -0.4, 0.4);
    drive.drive(distOutput, 0, yawOutput, false);
  }

  @Override
  public boolean isFinished() {
    if (lostTargetFrames >= 10) {
      return true;
    }
    return yawPID.atSetpoint() && distPID.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    drive.drive(0, 0, 0, false);
  }
}