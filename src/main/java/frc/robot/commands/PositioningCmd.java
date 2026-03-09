package frc.robot.commands;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.TagTracking;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import java.util.function.Supplier;

public class PositioningCmd extends Command {
  private final SwerveDrive drive;
  private final Supplier<Boolean> useLimelight;
  private final TagTracking[] limelights;
  
  public PositioningCmd(SwerveDrive driveSubsystem, Supplier<Boolean> useLimelight, TagTracking... limelights) {
    this.drive = driveSubsystem;
    this.useLimelight = useLimelight;
    this.limelights = limelights;
  }

  @Override
  public void execute() {
    SmartDashboard.putBoolean("vision/limelightEnabled", useLimelight.get());
    if (!useLimelight.get()) {
      return;
    }
    double yaw = drive.getGyroRotation2d().getDegrees();
    double yawRate = Math.toDegrees(drive.getRobotRelativeSpeeds().omegaRadiansPerSecond);

    for (TagTracking limelight : limelights) {
      limelight.setRobotOrientation(yaw, yawRate, 0, 0, 0, 0);

      if (limelight.hasTarget()) {
        double[] poseArray = limelight.getBotPoseArrayMegaTag2();
        double[] targetPoseRobot = limelight.getTargetPoseRobotSpace();

        if (poseArray.length >= 11 && targetPoseRobot.length >= 6) {
          double distance = poseArray[9];

          double trustValue = Math.min(0.4 + (distance * 0.6), 5.0);
          Pose2d visionPose = new Pose2d(poseArray[0], poseArray[1], Rotation2d.fromDegrees(poseArray[5]));
          double timestamp = Timer.getFPGATimestamp() - (poseArray[6] / 1000.0);
          
          drive.addVisionMeasurement(visionPose, timestamp, VecBuilder.fill(trustValue, trustValue, 9999999));
        }
      }
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}

