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
  private final Pose2d[] visionPoses;
  private final double[] lastTimestamps;
  private final double[] lastTrustValues;
  private final boolean[] lastHasTarget;

  public PositioningCmd(SwerveDrive driveSubsystem, TagTracking... limelights) {
    this.drive = driveSubsystem;
    this.limelights = limelights;

    int len = limelights.length;
    this.visionPoses = new Pose2d[len];
    this.lastTimestamps = new double[len];
    this.lastTrustValues = new double[len];
    this.lastHasTarget = new boolean[len];
  }

  public Pose2d[] updatePoses() {
    double yaw = drive.getPose2d().getRotation().getDegrees();
    double yawRate = Math.toDegrees(drive.getRobotRelativeSpeeds().omegaRadiansPerSecond);

    for (int i = 0; i < limelights.length; i++) {
      TagTracking limelight = limelights[i];
      limelight.setRobotOrientation(yaw, yawRate, 0, 0, 0, 0);
      lastHasTarget[i] = false; 

      if (limelight.hasTarget()) {
        double[] poseArray = limelight.getBotPoseArrayMegaTag2();
        double[] targetPoseRobot = limelight.getTargetPoseRobotSpace();

        if (poseArray.length >= 11 && targetPoseRobot.length >= 6) {
          double distance = poseArray[9];
          
          lastTrustValues[i] = Math.min(0.4 + (distance * 0.6), 5.0);
          visionPoses[i] = new Pose2d(poseArray[0], poseArray[1], Rotation2d.fromDegrees(poseArray[5]));
          lastTimestamps[i] = Timer.getFPGATimestamp() - (poseArray[6] / 1000.0);
          lastHasTarget[i] = true;
        }
      }

      if (!lastHasTarget[i]) {
        visionPoses[i] = new Pose2d();
      }
    }
    return visionPoses;
  }

  public void applyToDrive() {
    for (int i = 0; i < limelights.length; i++) {
      if (lastHasTarget[i]) {
        double trust = lastTrustValues[i];
        drive.addVisionMeasurement(visionPoses[i], lastTimestamps[i], VecBuilder.fill(trust, trust, 9999999));
      }
    }
  }

  @Override
  public void execute() {
    updatePoses();
    applyToDrive();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}