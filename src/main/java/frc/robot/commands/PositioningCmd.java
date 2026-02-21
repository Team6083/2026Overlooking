package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
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
