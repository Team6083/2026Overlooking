// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivebase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
  /** Creates a new SwerveDrive. */
  private final SwerveDriveKinematics kinematics;
  public SwerveModule frontLeft = new SwerveModule(
      25, 26, 33, 0.156494, true, false);
  public SwerveModule backLeft = new SwerveModule(
      23, 24, 34, 0.931152, true, false);
  public SwerveModule frontRight = new SwerveModule(
      27, 28, 32, 0.458008, true, false);
  public SwerveModule backRight = new SwerveModule(
      21, 22, 31, 0.746094, true, false);

  public SwerveDrive() {
    kinematics = new SwerveDriveKinematics(
        new Translation2d(+0.3, +0.3),
        new Translation2d(+0.3, -0.3),
        new Translation2d(-0.3, +0.3),
        new Translation2d(-0.3, -0.3));
  }

  public void drive(double vx, double vy, double omega) {

    ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, omega);

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

    frontLeft.setDesiredState(states[0]);
    frontRight.setDesiredState(states[1]);
    backLeft.setDesiredState(states[2]);
    backRight.setDesiredState(states[3]);
  }

  @Override
  public void periodic() {
    SmartDashboard.putData(backLeft.pid);
    SmartDashboard.putNumber("motorOutput", backLeft.turningMotor.get());
    // This method will be called once per scheduler run
  }
}
