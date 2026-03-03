// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ShooterComboCmd;
import frc.robot.commands.SwerveControlCmd;
import frc.robot.lib.TagTracking;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import frc.robot.subsystems.swervedrive.SwerveDriveFactory;

public class RobotContainer {
  private final TagTracking shooterTracker;
  private final SwerveDrive swerveDrive;
  private final CommandXboxController mainController = new CommandXboxController(0);
  private final ShooterSubsystem shooterSubsystem;
  private final TransportSubsystem transportSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final SendableChooser<Command> autoChooser;

  private Supplier<Double> magnification;
  private Supplier<Double> rotMagnification;

  public RobotContainer() {
    shooterTracker = new TagTracking("limelight-shooter");
    swerveDrive = SwerveDriveFactory.createSwerveDrive(
        SwerveDriveFactory.SwerveImplementation.WPILIB,
        SwerveDriveFactory.RobotVariant.COMPETITION);

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
    swerveDrive.setDefaultCommand(new SwerveControlCmd(swerveDrive, mainController, magnification, rotMagnification));
    mainController.start().onTrue(swerveDrive.zeroGyroCommand());
    // shooter
    mainController.rightBumper().whileTrue(new ShooterComboCmd(shooterSubsystem, transportSubsystem));
    mainController.x().toggleOnTrue(shooterSubsystem.shootCmd());
    // transport
    mainController.b().whileTrue(transportSubsystem.transportInCmd());
    // intake
    mainController.y().onTrue(intakeSubsystem.deployIntakeCmd());
    mainController.povDown().whileTrue(intakeSubsystem.manualDeployIntakeCmd());
    mainController.povUp().whileTrue(intakeSubsystem.manualRetractCmd());
    mainController.rightTrigger().whileTrue(intakeSubsystem.intakeCmd());
    mainController.leftTrigger().whileTrue(intakeSubsystem.reverseIntakeCmd());

    magnification = () -> mainController.leftBumper().getAsBoolean() ? 0.6 : 0.3;
    rotMagnification = () -> mainController.leftBumper().getAsBoolean() ? 0.8 : 0.4;
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}