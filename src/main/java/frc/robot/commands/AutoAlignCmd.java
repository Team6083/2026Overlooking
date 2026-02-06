package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.lib.TagTracking;

public class AutoAlignCmd extends Command {

  private final TagTracking vision;
  private final PIDController yawPID;
  private final PIDController distPID; 
  private int lostTargetFrames = 0;

  public AutoAlignCmd(TagTracking vision) {
    this.vision = vision;
    this.yawPID = new PIDController(VisionConstants.yawP, 0, 0);
    yawPID.setTolerance(VisionConstants.yawTolerance);
    this.distPID = new PIDController(VisionConstants.distP, 0, 0);
    distPID.setTolerance(VisionConstants.distanceTolerance);
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
      return;
    }

    lostTargetFrames = 0;
    double yawOutput = yawPID.calculate(vision.getTx(), 0);
    yawOutput = MathUtil.clamp(yawOutput, -0.5, 0.5);
    double distOutput = distPID.calculate(vision.get3dTz(), VisionConstants.idealDistance);
    distOutput = MathUtil.clamp(distOutput, -VisionConstants.maxForwardSpeed, VisionConstants.maxForwardSpeed);
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
  }
}