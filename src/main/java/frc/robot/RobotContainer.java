// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveControlCmd;
import frc.robot.lib.TagTracking;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import frc.robot.subsystems.swervedrive.YagslSwerve;

public class RobotContainer {
  private final TagTracking shooterTracker;
  private final SwerveDrive swerveDrive;
  private final CommandXboxController mainController = new CommandXboxController(0);
  private final SendableChooser<Command> autoChooser;
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  public RobotContainer() {
    shooterTracker = new TagTracking("limelight-shooter");
    swerveDrive = new YagslSwerve(new File(Filesystem.getDeployDirectory(), "swerve"));
    intakeSubsystem = new IntakeSubsystem();
    shooterSubsystem = new ShooterSubsystem();

    Auto.configureAutoBuilder(swerveDrive);
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("autoChooser", autoChooser);

    registerCommand();
    configureBindings();
  }

  private void registerCommand() {
    NamedCommands.registerCommand("deployIntake", intakeSubsystem.deployIntakeCmd());
    NamedCommands.registerCommand("intake", intakeSubsystem.intakeCmd());
    NamedCommands.registerCommand("shoot", shooterSubsystem.shootCmd());

  }

  private void configureBindings() {
    swerveDrive.setDefaultCommand(new SwerveControlCmd(swerveDrive, mainController));
    mainController.button(8).onTrue(swerveDrive.zeroGyroCommand());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}