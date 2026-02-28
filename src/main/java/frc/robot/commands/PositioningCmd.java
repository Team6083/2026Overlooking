package frc.robot.commands;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

  public Pose2d[] updatePoses() {
    Pose2d[] visionPoses = new Pose2d[limelights.length];
    Pose2d currentRobotPose = drive.getPose2d();
    ChassisSpeeds robotSpeeds = drive.getRobotRelativeSpeeds();

    for (int i = 0; i < limelights.length; i++) {
      TagTracking limelight = limelights[i];
      visionPoses[i] = new Pose2d();

      limelight.setRobotOrientation(
          currentRobotPose.getRotation().getDegrees(), 
          Math.toDegrees(robotSpeeds.omegaRadiansPerSecond), 
          0, 0, 0, 0 
      );

      if (limelight.hasTarget()) {
        double[] poseArray = limelight.getBotPoseArrayMegaTag2();
        double[] targetPoseRobot = limelight.getTargetPoseRobotSpace();

        if (poseArray.length >= 11 && targetPoseRobot.length >= 6) {
          double distance = poseArray[9];
          double trustValue = Math.min(0.4 + (distance * 0.6), 5.0); 

          Pose2d visionPose = new Pose2d(poseArray[0], poseArray[1], Rotation2d.fromDegrees(poseArray[5]));
          double timestamp = Timer.getFPGATimestamp() - (poseArray[6] / 1000.0);

          drive.addVisionMeasurement(visionPose, timestamp, VecBuilder.fill(trustValue, trustValue, trustValue));
          
          visionPoses[i] = visionPose;
        }
      }
    }
    return visionPoses;
  }

  @Override
  public void execute() {
    updatePoses();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
