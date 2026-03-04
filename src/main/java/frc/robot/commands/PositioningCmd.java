package frc.robot.commands;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
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
        double[] targetPoseRobot = limelight.getTargetPoseRobotSpace();

        if (poseArray.length >= 7 && targetPoseRobot.length >= 6) {
          double distance = Math.sqrt(Math.pow(targetPoseRobot[0], 2) + Math.pow(targetPoseRobot[2], 2));
          
          double trustValue = Math.min(0.4 + (distance * 0.6), 5.0);
          Pose2d visionPose = new Pose2d(poseArray[0], poseArray[1], Rotation2d.fromDegrees(poseArray[5]));
          double timestamp = Timer.getFPGATimestamp() - (poseArray[6] / 1000.0);
          
          // 核心任務：更新底層定位
          drive.addVisionMeasurement(visionPose, timestamp, VecBuilder.fill(trustValue, trustValue, trustValue));
        }
      }
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}

