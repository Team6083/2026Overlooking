package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.TagTracking;
import frc.robot.subsystems.swervedrive.SwerveDrive;

public class PositioningCmd extends Command {
  private final SwerveDrive drive;
  private final TagTracking[] limelights;

  public PositioningCmd(SwerveDrive driveSubsystem, TagTracking... limelights) {
    this.drive = driveSubsystem;
    this.limelights = limelights;
  }

  @Override
  public void execute() {
    for (TagTracking limelight : limelights) {
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

          SmartDashboard.putNumber(limelight.getName() + "/X", poseArray[0]);
          SmartDashboard.putNumber(limelight.getName() + "/Y", poseArray[1]);
        }
      }
    }

    @Override
    public void execute() {
        if (limelight.hasTarget()) {
            Pose2d visionPose = limelight.getBotPoseAsPose2d();
            double timestamp = limelight.getTimestampSeconds();
            if (visionPose != null) {
                drive.addVisionMeasurement(visionPose, timestamp);
            }
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
