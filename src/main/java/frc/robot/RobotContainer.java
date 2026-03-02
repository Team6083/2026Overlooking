// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoAlignCmd;
import frc.robot.commands.ShooterComboCmd;
import frc.robot.commands.SwerveControlCmd;
import frc.robot.lib.TagTracking;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;
// import frc.robot.subsystems.swervedrive.WpilibSwerveDrive;
import frc.robot.subsystems.swervedrive.YagslSwerve;
import java.io.File;

public class RobotContainer {
  private final TagTracking shooterTracker;
  private final SwerveDrive swerveDrive;
  private final CommandXboxController mainController = new CommandXboxController(0);
  private final ShooterSubsystem shooterSubsystem;
  private final TransportSubsystem transportSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    shooterTracker = new TagTracking("limelight-shooter");
    swerveDrive = new YagslSwerve(new File(Filesystem.getDeployDirectory(), "swerve"));
    // swerveDrive = new WpilibSwerveDrive();

    shooterSubsystem = new ShooterSubsystem();
    transportSubsystem = new TransportSubsystem();
    intakeSubsystem = new IntakeSubsystem();

    Auto.configureAutoBuilder(swerveDrive);

    registerCommand();

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("autoChooser", autoChooser);

    configureBindings();

  }

  private void registerCommand() {
    NamedCommands.registerCommand("deployIntake", intakeSubsystem.deployIntakeCmd());
    NamedCommands.registerCommand("intake", intakeSubsystem.intakeCmd());
    NamedCommands.registerCommand("shoot", new ShooterComboCmd(shooterSubsystem, transportSubsystem).withTimeout(5));
  }

  private void configureBindings() {
    // swerve drive
    swerveDrive.setDefaultCommand(new SwerveControlCmd(swerveDrive, mainController));
    mainController.start().onTrue(swerveDrive.zeroGyroCommand());
    // shooter
    mainController.a().whileTrue(new ShooterComboCmd(shooterSubsystem, transportSubsystem));
    mainController.x().toggleOnTrue(shooterSubsystem.shootCmd());
    // transport
    mainController.b().whileTrue(transportSubsystem.transportInCmd());
    // intake
    mainController.y().onTrue(intakeSubsystem.deployIntakeCmd());
    mainController.povDown().whileTrue(intakeSubsystem.manualDeployIntakeCmd());
    mainController.povUp().whileTrue(intakeSubsystem.manualRetractCmd());
    mainController.rightTrigger().whileTrue(intakeSubsystem.intakeCmd());
    mainController.leftTrigger().whileTrue(intakeSubsystem.reverseIntakeCmd());

    mainController.rightBumper().whileTrue(
        Commands.either(
            shooterSubsystem.shootCmd()
                .alongWith(new AutoAlignCmd(shooterTracker, swerveDrive))
                .andThen(new ShooterComboCmd(shooterSubsystem, transportSubsystem)),
            Commands.none(),
            () -> shooterTracker.isHubTag()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void autoInit() {
    swerveDrive.zeroGyro();
  }
}