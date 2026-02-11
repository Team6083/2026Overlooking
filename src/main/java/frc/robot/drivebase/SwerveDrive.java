// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivebase;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveBaseConstant;

public class SwerveDrive extends SubsystemBase {
  /** Creates a new SwerveDrive. */
  private final SwerveDriveKinematics kinematics;
  public SwerveModule frontLeft = new SwerveModule(
      25, 26, 33, 0.321533, true, true, "frontLeft");
  public SwerveModule backLeft = new SwerveModule(
      23, 24, 34, -0.434814, true, true, "backLeft");
  public SwerveModule frontRight = new SwerveModule(

      27, 28, 32, 0.499756, true, true, "frontRight");
  public SwerveModule backRight = new SwerveModule(
      21, 22, 31, 0.314453, true, true, "backRight");

  private final AHRS gyro;

  private SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
  private final StructArrayPublisher<SwerveModuleState> swerveDesiredStatePublisher = NetworkTableInstance
      .getDefault().getStructArrayTopic("DesiredStates", SwerveModuleState.struct).publish();
  private final StructArrayPublisher<SwerveModuleState> swerveCurrentStatePublisher = NetworkTableInstance
      .getDefault().getStructArrayTopic("CurrentStates", SwerveModuleState.struct).publish();

  public SwerveDrive() {
    kinematics = new SwerveDriveKinematics(
        new Translation2d(+0.27, +0.27),
        new Translation2d(+0.27, -0.27),
        new Translation2d(-0.27, +0.27),
        new Translation2d(-0.27, -0.27));
    gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
    gyro.reset();

    swerveModuleStates[0] = new SwerveModuleState();
    swerveModuleStates[1] = new SwerveModuleState();
    swerveModuleStates[2] = new SwerveModuleState();
    swerveModuleStates[3] = new SwerveModuleState();
  }

  public void drive(double vx, double vy, double omega, boolean feildRelative) {
    ChassisSpeeds speeds = feildRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega,
        gyro.getRotation2d()) : new ChassisSpeeds(vx, vy, omega);

    swerveModuleStates = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, 4);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  public Command resetGyroCmd() {
    Command cmd = runOnce(() -> gyro.reset());
    return cmd;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("gyro", gyro.getAngle());
    swerveDesiredStatePublisher.set(swerveModuleStates);
  }
}
