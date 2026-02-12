// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ShooterComboCmd;
import frc.robot.commands.SwerveControlCmd;
import frc.robot.lib.TagTracking;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import frc.robot.subsystems.swervedrive.YagslSwerve;
import com.pathplanner.lib.auto.AutoBuilder;
import java.io.File;

public class RobotContainer {
  private final TagTracking shooterTracker;
  private final SwerveDrive swerveDrive;
  private final CommandXboxController mainController = new CommandXboxController(0);
  private final SendableChooser<Command> autoChooser;
  private final ShooterSubsystem shooterSubsystem;
  private final TransportSubsystem transportSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  
  public RobotContainer() {
    shooterTracker = new TagTracking("limelight-shooter");
    swerveDrive = new YagslSwerve(new File(Filesystem.getDeployDirectory(), "swerve"));

    intakeSubsystem = new IntakeSubsystem();
    shooterSubsystem = new ShooterSubsystem();
    transportSubsystem = new TransportSubsystem();

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
    mainController.start().onTrue(swerveDrive.zeroGyroCommand());
    mainController.a().whileTrue(new ShooterComboCmd(shooterSubsystem, transportSubsystem));
    mainController.x().toggleOnTrue(shooterSubsystem.shootCmd());
    mainController.b().whileTrue(transportSubsystem.transportInCmd());
    mainController.y().onTrue(intakeSubsystem.deployIntakeCmd());
    mainController.povDown().whileTrue(intakeSubsystem.manualDeployIntakeCmd());
    mainController.povUp().whileTrue(intakeSubsystem.manualRetractCmd());
    mainController.rightTrigger().whileTrue(intakeSubsystem.intakeCmd());
    mainController.leftTrigger().whileTrue(intakeSubsystem.reverseIntakeCmd());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}