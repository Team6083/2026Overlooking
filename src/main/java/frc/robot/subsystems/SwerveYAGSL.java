package frc.robot.subsystems;

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
import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveYAGSL extends SubsystemBase implements frc.robot.subsystems.swervedrive.SwerveDrive {
  /** Creates a new SwerveDrive. */
  private final SwerveDrive swerveDrive;

  StructPublisher<Pose2d> currentPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("MyPose", Pose2d.struct).publish();
  StructArrayPublisher<Pose2d> arrayPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("MyPoseArray", Pose2d.struct).publish();

  private final List<Pose2d> poseHistory = new ArrayList<>();

  private final SlewRateLimiter limiterX = new SlewRateLimiter(3);
  private final SlewRateLimiter limiterY = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  public SwerveYAGSL(File directory) {
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

    swerveDrive.setHeadingCorrection(true);
    swerveDrive.setCosineCompensator(false);
    swerveDrive.setAngularVelocityCompensation(false, false, 0.1); // test
    swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
    swerveDrive.setChassisDiscretization(true, 0.02);
    swerveDrive.setMotorIdleMode(true);

    swerveDrive.zeroGyro();
  }

  @Override
  public void drive(double translationX, double translationY, double angularRotationX, boolean fieldRelative) {
    swerveDrive.drive(new Translation2d(translationX, translationY),
        angularRotationX * swerveDrive.getMaximumChassisAngularVelocity(),
        fieldRelative, false);
  }

  @Override
  public Command driveCommand(Supplier<Double> translationX, Supplier<Double> translationY,
      Supplier<Double> angularRotationX, boolean fieldRelative) {
    Command cmd = run(() -> swerveDrive.drive(new Translation2d(
        -limiterX.calculate(translationX.get()) * swerveDrive.getMaximumChassisVelocity(),
        -limiterY.calculate(translationY.get()) * swerveDrive.getMaximumChassisVelocity()),
        -rotLimiter
            .calculate(angularRotationX.get()) * swerveDrive.getMaximumChassisAngularVelocity(),
        fieldRelative, false));
    return cmd;
  }

  @Override
  public Command driveCommand(double translationX, double translationY, double angularRotationX,
      boolean fieldRelative) {
    Command cmd = run(
        () -> this.drive(translationX, translationY, angularRotationX, fieldRelative));
    return cmd;
  }

  @Override
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  @Override
  public Pose2d getPose2d() {
    return swerveDrive.getPose();
  }

  @Override
  public void resetPose(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  @Override
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return swerveDrive.getRobotVelocity();
  }

  @Override
  public Command zeroGyroCommand() {
    Command cmd = runOnce(() -> this.zeroGyro());
    cmd.setName("zeroGyroCommand");
    return cmd;
  }

  public double getGyroHeading() {
    double gyroHeading = swerveDrive.getYaw().getDegrees() % 360;
    if (gyroHeading < 0) {
      gyroHeading += 360;
    }
    return gyroHeading;
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
