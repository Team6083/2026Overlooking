package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import frc.robot.lib.TagTracking;

public class PositioningCmd extends Command {
  private final SwerveDrive drive;
  private final TagTracking limelight;

  public PositioningCmd(SwerveDrive driveSubsystem, TagTracking limelightSystem) {
    this.drive = driveSubsystem;
    this.limelight = limelightSystem;
  }

  @Override
  public void execute() {
    if (limelight.hasTarget()) {
      double[] poseArray = limelight.getBotPoseArray();
      if (poseArray.length >= 7) {
        Pose2d visionPose = new Pose2d(
          poseArray[0],
          poseArray[1],
          Rotation2d.fromDegrees(poseArray[5])
        );
        double timestamp = Timer.getFPGATimestamp() - (poseArray[6] / 1000.0);
        drive.addVisionMeasurement(visionPose, timestamp);
      }
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}