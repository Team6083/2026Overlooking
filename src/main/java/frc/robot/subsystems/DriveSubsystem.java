package frc.robot.subsystems;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
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

  private final SlewRateLimiter xLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

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
    swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
    swerveDrive.setChassisDiscretization(false, 0.02);
    swerveDrive.setMotorIdleMode(true);

    swerveDrive.zeroGyro();
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier angularRotationX) {
    Command cmd = run(() -> swerveDrive.drive(new Translation2d(
        xLimiter.calculate(translationX.getAsDouble()) * swerveDrive.getMaximumChassisVelocity(),
        yLimiter.calculate(translationY.getAsDouble()) * swerveDrive.getMaximumChassisVelocity()),
        rotLimiter
            .calculate(angularRotationX.getAsDouble()) * swerveDrive.getMaximumChassisAngularVelocity(),
        true, false));
    return cmd;
  }

  public Command driveCommand(double xSpeed, double ySpeed,
      double rotSpeed, boolean fieldRelative) {
    Command cmd = run(() -> swerveDrive.drive(new Translation2d(xSpeed, ySpeed), rotSpeed, fieldRelative, false));
    return cmd;
  }

  public Pose2d getPose2d() {
    return swerveDrive.getPose();
  }

  public double getGyroHeading() {
    double gyroHeading = swerveDrive.getYaw().getDegrees() % 360;
    if (gyroHeading < 0) {
      gyroHeading += 360;
    }
    return gyroHeading;
  }

  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  public Command zeroGyroCmd() {
    Command cmd = runOnce(() -> swerveDrive.zeroGyro());
    cmd.setName("zeroGyroCmd");
    return cmd;
  }

  public Command centerModulesCmd() {
    Command cmd = runOnce(() -> Arrays.asList(swerveDrive.getModules())
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
    var gyro = swerveDrive.getGyro().getIMU();
    boolean gyroIsConnected = false;
    if (gyro instanceof com.studica.frc.AHRS) {
      gyroIsConnected = ((com.studica.frc.AHRS) gyro).isConnected();
    }

    SmartDashboard.putNumber("gyroHeading", getGyroHeading());
    SmartDashboard.putBoolean("gyroIsConnected", gyroIsConnected);

    Pose2d currentPose = getPose2d();
    poseHistory.add(currentPose);
    currentPosePublisher.set(getPose2d());
    if (poseHistory.size() > 50) {
      poseHistory.remove(0);
    }
    arrayPublisher.set(poseHistory.toArray(new Pose2d[0]));
    SwerveDriveTelemetry.updateData();
  }
}
