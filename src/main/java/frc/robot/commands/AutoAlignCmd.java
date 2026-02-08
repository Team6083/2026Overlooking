package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.lib.TagTracking;
import frc.robot.subsystems.swervedrive.SwerveDrive;

public class AutoAlignCmd extends Command {

  private final TagTracking vision;
  private final SwerveDrive m_drive;
  private final PIDController yawPID;
  private final PIDController distPID; 
  private int lostTargetFrames = 0;

  public AutoAlignCmd(TagTracking vision, SwerveDrive drive) {
    this.vision = vision;
    this.m_drive = drive;
    this.yawPID = new PIDController(VisionConstants.yawP, 0, 0);
    yawPID.setTolerance(VisionConstants.yawTolerance);
    this.distPID = new PIDController(VisionConstants.distP, 0, 0);
    distPID.setTolerance(VisionConstants.distanceTolerance);
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
      m_drive.drive(0, 0, 0, false);
      return;
    }

    lostTargetFrames = 0;
    double yawOutput = yawPID.calculate(vision.getTx(), 0);
    yawOutput = MathUtil.clamp(yawOutput, -0.5, 0.5);
    double distOutput = distPID.calculate(vision.get3dTz(), VisionConstants.idealDistance);
    distOutput = MathUtil.clamp(distOutput, -VisionConstants.maxForwardSpeed, VisionConstants.maxForwardSpeed);

    m_drive.drive(distOutput, 0, yawOutput, false);
  }

  @Override
  public boolean isFinished() {
    if (lostTargetFrames >= VisionConstants.maxLostFrames) {
      return true;
    }
    return yawPID.atSetpoint() && distPID.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false);
  }
}