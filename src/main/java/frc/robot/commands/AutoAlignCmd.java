package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.TagTracking;
import frc.robot.subsystems.swervedrive.SwerveDrive;

public class AutoAlignCmd extends Command {

  private final TagTracking tagTracking;
  private final SwerveDrive drive;
  private final PIDController yawPID;
  private final PIDController tzPID;
  private final PIDController txPID;

  private final Debouncer targetDebouncer;
  private boolean isTargetValid;

  public AutoAlignCmd(TagTracking vision, SwerveDrive drive) {
    this.tagTracking = vision;
    this.drive = drive;
    yawPID = new PIDController(0.05, 0, 0);
    yawPID.setTolerance(1);
    tzPID = new PIDController(1, 0, 0.1);
    tzPID.setTolerance(0.15);
    txPID = new PIDController(1, 0, 0.1);
    txPID.setTolerance(0.15);
    targetDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kFalling);
    addRequirements(drive);

    SmartDashboard.putBoolean("TagTrackingHasTag", vision.hasTarget());
    SmartDashboard.putBoolean("TagTrackingIsHubTag", vision.isHubTag());
    SmartDashboard.putData("yawPID", yawPID);
    SmartDashboard.putData("TzPID", tzPID);
    SmartDashboard.putData("TxPID", txPID);
  }

  @Override
  public void initialize() {
    yawPID.reset();
    tzPID.reset();
  }

  @Override
  public void execute() {
    isTargetValid = targetDebouncer.calculate(tagTracking.hasTarget() && tagTracking.isHubTag());

    double yawOutput = 0;
    double tzOutput = 0;
    double txOutput = 0;

    if (isTargetValid) {
      yawOutput = yawPID.calculate(tagTracking.get3dYaw(), 0);
      yawOutput = MathUtil.clamp(yawOutput, -1, 1);

      tzOutput = tzPID.calculate(tagTracking.get3dTz(), 1.6);
      tzOutput = MathUtil.clamp(tzOutput, -1.7, 1.5);

      txOutput = -txPID.calculate(tagTracking.get3dTx(), 0);
      txOutput = MathUtil.clamp(txOutput, -1.5, 1.5);

      drive.drive(tzOutput, txOutput, yawOutput, false);
    }
  }

  @Override
  public boolean isFinished() {
    if (!isTargetValid) {
      return true;
    }
    return yawPID.atSetpoint() && tzPID.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    drive.drive(0, 0, 0, false);
  }
}