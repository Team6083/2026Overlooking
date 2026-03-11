// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrsConstants;
import frc.robot.subsystems.swervedrive.SwerveDrive;

public class DrsSubsystem extends SubsystemBase {
  /** Creates a new servoMotorSubsystem. */
  private final Servo drs;
  private final SwerveDrive swerveDrive;
  private final Supplier<Boolean> isAutoDrs;
  private boolean isManual = false;

  public DrsSubsystem(SwerveDrive swerveDrive, Supplier<Boolean> isAutoDrs) {
    drs = new Servo(DrsConstants.servoMotorChannel);
    this.swerveDrive = swerveDrive;
    this.isAutoDrs = isAutoDrs;
  }

  public void downDrs() {
    drs.setPosition(DrsConstants.downPosition);
  }

  public void upDrs() {
    drs.setPosition(DrsConstants.upPosition);
  }

  public Boolean isUnderBlueLeftTrench() {
    double trenchMinX = 3.9;
    double trenchMaxX = 5.4;
    double trenchMinY = 6.5;
    double trenchMaxY = 8.0;

    if (swerveDrive.getPose2d().getX() > trenchMinX && swerveDrive.getPose2d().getX() < trenchMaxX
        && swerveDrive.getPose2d().getY() > trenchMinY && swerveDrive.getPose2d().getY() < trenchMaxY) {
      return true;
    }
    return false;
  }

  public Boolean isUnderBlueRightTrench() {
    double trenchMinX = 3.9;
    double trenchMaxX = 5.4;
    double trenchMinY = 0;
    double trenchMaxY = 1.5;

    if (swerveDrive.getPose2d().getX() > trenchMinX && swerveDrive.getPose2d().getX() < trenchMaxX
        && swerveDrive.getPose2d().getY() > trenchMinY && swerveDrive.getPose2d().getY() < trenchMaxY) {
      return true;
    }
    return false;
  }

  public Boolean isUnderRedLeftTrench() {
    double trenchMinX = 11.2;
    double trenchMaxX = 12.7;
    double trenchMinY = 0;
    double trenchMaxY = 1.5;

    if (swerveDrive.getPose2d().getX() > trenchMinX && swerveDrive.getPose2d().getX() < trenchMaxX
        && swerveDrive.getPose2d().getY() > trenchMinY && swerveDrive.getPose2d().getY() < trenchMaxY) {
      return true;
    }
    return false;
  }

  public Boolean isUnderRedRightTrench() {
    double trenchMinX = 11.4;
    double trenchMaxX = 12.7;
    double trenchMinY = 6.5;
    double trenchMaxY = 8;

    if (swerveDrive.getPose2d().getX() > trenchMinX && swerveDrive.getPose2d().getX() < trenchMaxX
        && swerveDrive.getPose2d().getY() > trenchMinY && swerveDrive.getPose2d().getY() < trenchMaxY) {
      return true;
    }
    return false;
  }

  public Command downDrsCmd() {
    Command cmd = runOnce(() -> {
      isManual = true;
      downDrs();
    });
    cmd.setName("downDrsCmd");
    return cmd;
  }

  public Command upDrsCmd() {
    Command cmd = runOnce(()-> {
      isManual = true;
      upDrs();
    });
    cmd.setName("upDrsCmd");
    return cmd;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (isAutoDrs.get()) {
      if (!isManual) {
        if (isUnderRedLeftTrench() || isUnderRedRightTrench() || isUnderBlueLeftTrench() || isUnderBlueRightTrench()) {
          downDrs();
        } else {
          upDrs();
        }
      }
    }
    SmartDashboard.putNumber("servoMotorAngle", drs.getAngle());
    SmartDashboard.putNumber("servoMotorPosition", drs.get());
  }
}
