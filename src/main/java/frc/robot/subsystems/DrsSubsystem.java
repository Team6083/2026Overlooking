// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrsConstants;
import frc.robot.Constants.FieldZones;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import java.util.function.Supplier;

public class DrsSubsystem extends SubsystemBase {
  /** Creates a new servoMotorSubsystem. */
  private final Servo drs;
  private final SwerveDrive swerveDrive;
  private boolean shouldDrsUp;
  private final Debouncer targetDebouncer;
  private final Supplier<Boolean> isAutoDrs;

  public DrsSubsystem(SwerveDrive swerveDrive, Supplier<Boolean> isAutoDrs) {
    drs = new Servo(DrsConstants.servoMotorChannel);
    this.swerveDrive = swerveDrive;
    this.isAutoDrs = isAutoDrs;
    this.targetDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kFalling);
  }

  public Boolean isUnderTrench() {
    return targetDebouncer.calculate(FieldZones.trenchZone.contains(swerveDrive.getPose2d().getTranslation()));
  }

  public void setTargetPosition(boolean shouldDrsUp) {
    this.shouldDrsUp = shouldDrsUp;
  }

  public Command downDrsCmd() {
    Command cmd = runOnce(() -> setTargetPosition(false));
    cmd.setName("downDrsCmd");
    return cmd;
  }

  public Command upDrsCmd() {
    Command cmd = runOnce(() -> setTargetPosition(true));
    cmd.setName("upDrsCmd");
    return cmd;
  }

  @Override
  public void periodic() {
    if (isAutoDrs.get() && isUnderTrench()) {
      drs.setPosition(DrsConstants.downPosition);
    } else {
      if (shouldDrsUp) {
        drs.setPosition(DrsConstants.upPosition);
      } else {
        drs.setPosition(DrsConstants.downPosition);
      }
    }

    SmartDashboard.putNumber("drs/drsAngle", drs.getAngle());
    SmartDashboard.putNumber("drs/drsPosition", drs.get());
  }
}
