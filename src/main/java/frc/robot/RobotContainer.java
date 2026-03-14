// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AimAssistCmd;
import frc.robot.commands.PositioningCmd;
import frc.robot.commands.ShooterComboCmd;
import frc.robot.commands.SwerveControlCmd;
import frc.robot.lib.TagTracking;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrsSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import frc.robot.subsystems.swervedrive.SwerveDriveFactory;
import java.util.function.Supplier;

public class RobotContainer {
  private final TagTracking shooterTracker;
  private final TagTracking backTracker;
  private final SwerveDrive swerveDrive;
  private final CommandXboxController mainController = new CommandXboxController(0);
  private final CommandGenericHID controlPanel = new CommandGenericHID(1);
  private final ClimberSubsystem climberSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final TransportSubsystem transportSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final SendableChooser<Command> autoChooser;
  private final DrsSubsystem drsSubsystem;
  private final PositioningCmd positioningCmd;
  private final StructArrayPublisher<Pose2d> visionPosePublisher = NetworkTableInstance
      .getDefault().getStructArrayTopic("visionPoses", Pose2d.struct).publish();

  private Supplier<Boolean> shouldSprint = () -> mainController.leftBumper().getAsBoolean();
  private Supplier<Boolean> shouldAutoDrs = () -> controlPanel.button(12).getAsBoolean();

  public RobotContainer() {
    shooterTracker = new TagTracking("limelight-shooter");
    backTracker = new TagTracking("limelight-back");
    swerveDrive = SwerveDriveFactory.createSwerveDrive(
        SwerveDriveFactory.SwerveImplementation.WPILIB,
        SwerveDriveFactory.RobotVariant.COMPETITION);
    positioningCmd = new PositioningCmd(swerveDrive, shooterTracker, backTracker);

    climberSubsystem = new ClimberSubsystem();
    shooterSubsystem = new ShooterSubsystem();
    transportSubsystem = new TransportSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    feederSubsystem = new FeederSubsystem();
    drsSubsystem = new DrsSubsystem(swerveDrive, shouldAutoDrs);

    SmartDashboard.putData("PositioningCmd", positioningCmd);

    Auto.configureAutoBuilder(swerveDrive);

    registerCommand();

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("autoChooser", autoChooser);

    configureBindings();

  }

  public void updateVision() {
    TagTracking[] trackers = { shooterTracker, backTracker };
    Pose2d[] visionPoses = new Pose2d[trackers.length];

    for (int i = 0; i < trackers.length; i++) {
      if (trackers[i].hasTarget()) {
        double[] poseArray = trackers[i].getBotPoseArrayMegaTag2();
        if (poseArray.length >= 11) {
          visionPoses[i] = new Pose2d(poseArray[0], poseArray[1], Rotation2d.fromDegrees(poseArray[5]));
        } else {
          visionPoses[i] = new Pose2d();
        }
      } else {
        visionPoses[i] = new Pose2d();
      }
    }
    visionPosePublisher.set(visionPoses);
  }

  private final Field2d field = new Field2d();

  public void putRobotPoseOnDashboard() {
    field.setRobotPose(swerveDrive.getPose2d());
    SmartDashboard.putData("Robot Pose", field);
  }

  private void registerCommand() {
    NamedCommands.registerCommand("deployIntake", intakeSubsystem.deployIntakeCmd());
    NamedCommands.registerCommand("intake", intakeSubsystem.intakeCmd());
    NamedCommands.registerCommand("previousShoot", shooterSubsystem.shootCmd());
    NamedCommands.registerCommand("shoot",
        new ShooterComboCmd(
            swerveDrive, shooterSubsystem,
            transportSubsystem, feederSubsystem, intakeSubsystem,
            () -> true)
            .withTimeout(4));
  }

  private Command shooterComboCmd() {
    Command cmd = new ShooterComboCmd(
        swerveDrive, shooterSubsystem,
        transportSubsystem, feederSubsystem,
        intakeSubsystem, controlPanel.button(11));
    cmd.setName("shooterComboCmd");
    return cmd;
  }

  private void configureBindings() {
    // position tracking
    controlPanel.button(9).whileTrue(positioningCmd);
    shooterSubsystem.setDefaultCommand(shooterSubsystem.shootCmd(2000));

    // swerve drive
    swerveDrive.setDefaultCommand(new SwerveControlCmd(swerveDrive, mainController, shouldSprint));
    mainController.start().onTrue(Commands.runOnce(() -> {
      swerveDrive.zeroGyro();
      swerveDrive.resetPose(new Pose2d(swerveDrive.getPose2d().getTranslation(), Rotation2d.fromDegrees(0)));
    }));

    // shooter
    var shooterComboWithAimAssistCmd = new AimAssistCmd(swerveDrive, mainController, shouldSprint)
        .alongWith(shooterComboCmd());
    shooterComboWithAimAssistCmd.setName("shooterComboWithAimAssistCmd");

    var shooterComboWithSwerveControlCmd = new SwerveControlCmd(swerveDrive, mainController, shouldSprint)
        .alongWith(shooterComboCmd());

    controlPanel.button(1)
        .whileTrue(Commands.either(
            shooterComboWithAimAssistCmd,
            shooterComboWithSwerveControlCmd,
            controlPanel.button(10)));

    controlPanel.button(3).whileTrue(shooterSubsystem.shootCmd());

    // transport
    controlPanel.button(8).whileTrue(transportSubsystem.transportInCmd()
        .alongWith(feederSubsystem.feedInCmd())
        .alongWith(intakeSubsystem.intakeCmd()));

    // intake
    controlPanel.button(7).whileTrue(intakeSubsystem.reverseIntakeCmd());
    controlPanel.button(6).whileTrue(intakeSubsystem.intakeCmd());

    mainController.povUp().whileTrue(intakeSubsystem.manualRetractCmd());
    mainController.povDown().whileTrue(intakeSubsystem.manualDeployCmd());
    mainController.y().onTrue(intakeSubsystem.retractIntakeCmd());
    mainController.a().onTrue(intakeSubsystem.deployIntakeCmd());

    // climber
    mainController.rightTrigger().whileTrue(climberSubsystem.climbUpCmd());
    mainController.leftTrigger().whileTrue(climberSubsystem.climbDownCmd());

    // DRS
    controlPanel.button(5).onTrue(drsSubsystem.upDrsCmd());
    controlPanel.button(4).onTrue(drsSubsystem.downDrsCmd());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}