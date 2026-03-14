// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import java.util.function.Supplier;

public class LockPoseCmd extends SwerveControlCmd {
  private final SwerveDrive swerveDrive;

  public LockPoseCmd(SwerveDrive swerveDrive, CommandXboxController mainController,
      Supplier<Boolean> shouldSprint) {
    super(swerveDrive, mainController, shouldSprint);
    this.swerveDrive = swerveDrive;
  }

  @Override
  public void execute() {
    if (shouldLock()) {
      swerveDrive.lockPose();
    } else {
      super.execute();
    }
  }

  public boolean shouldLock() {
    return calcSpeedX() < 0.1 &&
        calcSpeedY() < 0.1 &&
        calcRotSpeed() < 0.1;
  }
}