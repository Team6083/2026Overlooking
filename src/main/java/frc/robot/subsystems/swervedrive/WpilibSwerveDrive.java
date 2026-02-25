// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveBaseConstant;

public class WpilibSwerveDrive extends SubsystemBase implements frc.robot.subsystems.swervedrive.SwerveDrive {
  /** Creates a new SwerveDrive. */
  public SwerveModule frontLeft;
  public SwerveModule frontRight;
  public SwerveModule backLeft;
  public SwerveModule backRight;

  private final AHRS gyro;

  private final SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;

  private SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
  private final StructArrayPublisher<SwerveModuleState> swerveDesiredStatePublisher = NetworkTableInstance
      .getDefault().getStructArrayTopic("DesiredStates", SwerveModuleState.struct).publish();
  private final StructArrayPublisher<SwerveModuleState> swerveCurrentStatePublisher = NetworkTableInstance
      .getDefault().getStructArrayTopic("CurrentStates", SwerveModuleState.struct).publish();

  private final StructPublisher<Pose2d> currentPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("currentPose", Pose2d.struct).publish();

  public WpilibSwerveDrive(DriveBaseConstant driveBaseConstant) {
    frontLeft = new SwerveModule(driveBaseConstant.frontLeft());
    frontRight = new SwerveModule(driveBaseConstant.frontRight());
    backLeft = new SwerveModule(driveBaseConstant.backLeft());
    backRight = new SwerveModule(driveBaseConstant.backRight());

    gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
    gyro.reset();

    kinematics = new SwerveDriveKinematics(
        new Translation2d(+0.27, +0.27),
        new Translation2d(+0.27, -0.27),
        new Translation2d(-0.27, +0.27),
        new Translation2d(-0.27, -0.27));

    odometry = new SwerveDriveOdometry(
        kinematics,
        gyro.getRotation2d(),
        getSwerveModulePosition());

    swerveModuleStates[0] = new SwerveModuleState();
    swerveModuleStates[1] = new SwerveModuleState();
    swerveModuleStates[2] = new SwerveModuleState();
    swerveModuleStates[3] = new SwerveModuleState();
  }

  public SwerveModulePosition[] getSwerveModulePosition() {
    return new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
    };
  }

  private void updateOdometry() {
    odometry.update(
        gyro.getRotation2d(),
        getSwerveModulePosition());
  }

  @Override
  public void drive(double vx, double vy, double omega, boolean fieldRelative) {
    var alliance = DriverStation.getAlliance();
    var inverted = 1;
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      inverted = -1;
    }

    ChassisSpeeds speeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(vx * inverted, vy * inverted, omega),
            gyro.getRotation2d())
        : new ChassisSpeeds(vx, vy, omega);

    drive(speeds);
  }

  @Override
  public void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(states, 4.0);

    frontLeft.setDesiredState(states[0]);
    frontRight.setDesiredState(states[1]);
    backLeft.setDesiredState(states[2]);
    backRight.setDesiredState(states[3]);
  }

  @Override
  public void zeroGyro() {
    gyro.reset();
  }

  @Override
  public Command zeroGyroCommand() {
    Command cmd = runOnce(() -> zeroGyro());
    return cmd;
  }

  @Override
  public Pose2d getPose2d() {
    return odometry.getPoseMeters();
  }

  @Override
  public void resetPose(Pose2d pose) {
    odometry.resetPosition(
        gyro.getRotation2d(),
        getSwerveModulePosition(),
        pose);
  }

  @Override
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();

    SmartDashboard.putNumber("gyro", gyro.getRotation2d().getDegrees());
    swerveDesiredStatePublisher.set(swerveModuleStates);
    swerveCurrentStatePublisher
        .set(new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        });

    currentPosePublisher.set(getPose2d());
  }
}
