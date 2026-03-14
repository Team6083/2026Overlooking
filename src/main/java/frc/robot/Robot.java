// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.util.StatusLogger;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private boolean savelog = true;

  private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();

  private Timer gcTimer = new Timer();

  public Robot() {
    if (!savelog) {
      StatusLogger.disableAutoLogging();
    }

    m_robotContainer = new RobotContainer();
    gcTimer.start();
  }

  @Override
  public void robotInit() {
    m_robotContainer.putRobotPoseOnDashboard();

    if (savelog) {
      DataLogManager.start();
      DriverStation.startDataLog(DataLogManager.getLog());
    }

    SignalLogger.enableAutoLogging(savelog);

    ntInstance.getStringTopic("/Metadata/BuildDate").publish()
        .set(BuildConstants.BUILD_DATE);
    ntInstance.getStringTopic("/Metadata/GitBranch").publish()
        .set(BuildConstants.GIT_BRANCH);
    ntInstance.getStringTopic("/Metadata/GitDate").publish()
        .set(BuildConstants.GIT_DATE);
    ntInstance.getStringTopic("/Metadata/GitDirty").publish()
        .set(BuildConstants.DIRTY == 1 ? "Dirty!" : "Clean! Good job!");
    ntInstance.getStringTopic("/Metadata/GitSHA").publish()
        .set(BuildConstants.GIT_SHA);
    ntInstance.getStringTopic("/Metadata/GitBranch").publish()
        .set(BuildConstants.GIT_BRANCH);

    SmartDashboard.putString("Metadata/GitInfo", String.format("%s (%s), %s",
        BuildConstants.GIT_SHA,
        BuildConstants.GIT_BRANCH,
        BuildConstants.DIRTY == 1 ? "Dirty" : "Clean"));
    SmartDashboard.putString("Metadata/BuildDate", BuildConstants.BUILD_DATE);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.updateVision();
    if (gcTimer.advanceIfElapsed(5)) {
      System.gc();
    }
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
