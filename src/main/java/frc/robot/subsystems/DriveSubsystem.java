package frc.robot.subsystems;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new SwerveDrive. */
  private final SwerveDrive swerveDrive;

  StructPublisher<Pose2d> currentPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("MyPose", Pose2d.struct).publish();
  StructArrayPublisher<Pose2d> arrayPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("MyPoseArray", Pose2d.struct).publish();

  private final List<Pose2d> poseHistory = new ArrayList<>();

  public DriveSubsystem(File directory) {
    var alliance = DriverStation.getAlliance();
    boolean blueAlliance = alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Blue : true;
    Pose2d startingPose = blueAlliance ? new Pose2d(new Translation2d(1, 4),
        Rotation2d.fromDegrees(0))
        : new Pose2d(new Translation2d(16, 4),
            Rotation2d.fromDegrees(180));

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(4, startingPose);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    swerveDrive.setHeadingCorrection(false);
    swerveDrive.setCosineCompensator(false);
    swerveDrive.setAngularVelocityCompensation(false, false, 0.1); // test
    swerveDrive.setModuleEncoderAutoSynchronize(true, 1);
    swerveDrive.setChassisDiscretization(false, 0.02);
    swerveDrive.setMotorIdleMode(true);
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier angularRotationX) {
    Command cmd = run(() -> swerveDrive.drive(new Translation2d(
        translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
        translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
        Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
        true, false));
    return cmd;
  }

  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public Pose2d getPose2d() {
    return swerveDrive.getPose();
  }

  public Rotation2d getHeading() {
    return getPose2d().getRotation();
  }

  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  public Command resetOdomestryCmd(Pose2d initialHolonomicPose) {
    Command cmd = runOnce(() -> resetOdometry(initialHolonomicPose));
    cmd.setName("resetOdomestry");
    return cmd;
  }

  public Command zeroGyroCmd() {
    Command cmd = runOnce(() -> swerveDrive.zeroGyro());
    cmd.setName("resetOdomestry");
    return cmd;
  }

  public Command centerModulesCmd() {
    Command cmd = run(() -> Arrays.asList(swerveDrive.getModules())
        .forEach(it -> it.setAngle(0.0)));
    cmd.setName("centerModulesCmd");
    return cmd;
  }

  public Command lockPoseCmd() {
    Command cmd = run(() -> swerveDrive.lockPose());
    cmd.setName("lockPoseCmd");
    return cmd;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("gyroHeading", getHeading().getDegrees());
    Pose2d currentPose = getPose2d();
    poseHistory.add(currentPose);
    currentPosePublisher.set(getPose2d());
    arrayPublisher.set(poseHistory.toArray(new Pose2d[0]));
    SwerveDriveTelemetry.updateData();
  }
}
