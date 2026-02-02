// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.TagTracking;
import frc.robot.lib.VisionTelemetry;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

  private TagTracking shooterTracker;
  private VisionTelemetry visionTelemetry;

  private final Alert m_visionAlert = new Alert("Limelight 視覺系統關閉", AlertType.kWarning);

  private Timer gcTimer = new Timer();

  public Robot() {
    m_robotContainer = new RobotContainer();
    shooterTracker = new TagTracking();
    visionTelemetry = new VisionTelemetry(shooterTracker);
    gcTimer.start();
  }

  @Override
  public void robotInit() {
    SmartDashboard.setDefaultBoolean("disableLimelight", false);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("matchTime", DriverStation.getMatchTime());
    boolean isDisabled = SmartDashboard.getBoolean("disableLimelight", false);
    shooterTracker.setDisabled(isDisabled); 
    m_visionAlert.set(isDisabled);
    visionTelemetry.update();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {    
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}