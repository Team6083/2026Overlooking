// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.Vision;
import frc.robot.subsystems.swervedrive.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class VisionSwerveAutoCmd extends Command {
  private Vision vision;
  private SwerveDrive swerveDrive;
  private int currentLockedId = -1;
  private final double visionTurnKp = 0.6;
  private final double visionDriveKp = 0.5;

  /** Creates a new VisionSwerveAutoCmd. */
  public VisionSwerveAutoCmd(SwerveDrive swerveDrive, Vision vision) {
    this.swerveDrive = swerveDrive;
    this.vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentLockedId = -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] targetData = null;

    if (currentLockedId == -1) {
      // 回傳格式是 {x, y, id}
      double[] newTarget = vision.getClosestBallWithId();
      if (newTarget != null) {
        currentLockedId = (int) newTarget[2]; // 鎖定這顆球的 ID
        SmartDashboard.putNumber("目前工作ID", currentLockedId);
      } else {
        // 完全沒球，原地停車或原地旋轉搜尋
        swerveDrive.drive(0, 0, 0, false); // 停止
        return;
      }
      targetData = vision.getBallById(currentLockedId);
      if (targetData == null) {
        double[] rescueTarget = vision.getNextClosestBall(currentLockedId);

        if (rescueTarget != null) {
          currentLockedId = (int) rescueTarget[2]; // 無縫切換到新球
          SmartDashboard.putNumber("目前工作ID", currentLockedId);
          // 遞迴呼叫自己或是直接下一輪再處理，這裡簡單起見下一輪再跑
        } else { // 真的一顆球都沒了
          currentLockedId = -1; // 重置狀態
          swerveDrive.drive(0, 0, 0, false); // 停止
        }
        return; // 這一輪先結束
      } else {
        currentLockedId = -1; // 重置狀態(根本就沒球)
        swerveDrive.drive(0, 0, 0, false); // 停止
      }
      double tx = targetData[0];
      double ty = targetData[1];
      double turnOutput = tx * visionTurnKp;
      double driveOutput = (0.9 - ty) * visionDriveKp;
      driveOutput = MathUtil.clamp(driveOutput, -0.3, 0.3);
      swerveDrive.drive(driveOutput, 0, -turnOutput, false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerveDrive.drive(0, 0, 0, false);
    SmartDashboard.putNumber("目前工作ID", -1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
