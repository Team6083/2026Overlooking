package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
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
import java.util.Arrays;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class YagslSwerve extends SubsystemBase implements frc.robot.subsystems.swervedrive.SwerveDrive {
  /** Creates a new SwerveDrive. */
  private final SwerveDrive swerveDrive;

  StructPublisher<Pose2d> currentPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("currentPose", Pose2d.struct).publish();
  StructArrayPublisher<Pose2d> arrayPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("poseHistory", Pose2d.struct).publish();

  public YagslSwerve(File directory) {

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(4);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    swerveDrive.setHeadingCorrection(false);
    swerveDrive.setCosineCompensator(false);
    swerveDrive.setAngularVelocityCompensation(false, false, 0.1); // test
    swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
    swerveDrive.setChassisDiscretization(true, 0.02);
    swerveDrive.setMotorIdleMode(true);
  }

  @Override
  public void drive(double translationX, double translationY, double angularRotationX, boolean fieldRelative) {
    var alliance = DriverStation.getAlliance();
    var inverted = 1;
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      inverted = -1;
    }

    var directionMultiplier = fieldRelative ? inverted : 1;
    Translation2d finalTranslation = new Translation2d(
        translationX * directionMultiplier,
        translationY * directionMultiplier);

    swerveDrive.drive(
        finalTranslation,
        angularRotationX,
        fieldRelative, false);
  }

  @Override
  public void drive(ChassisSpeeds speeds) {
    swerveDrive.drive(speeds);
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

  public double getMaxSpeed() {
    return swerveDrive.getMaximumChassisVelocity();
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

    currentPosePublisher.set(getPose2d());
    SwerveDriveTelemetry.updateData();
  }
}
