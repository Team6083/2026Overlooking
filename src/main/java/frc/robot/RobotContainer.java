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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.PositioningCmd;
import frc.robot.commands.ShooterComboCmd;
import frc.robot.commands.SwerveControlCmd;
import frc.robot.lib.TagTracking;
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
  private final ShooterSubsystem shooterSubsystem;
  private final TransportSubsystem transportSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final SendableChooser<Command> autoChooser;
  private final PositioningCmd positioningCmd;
  private final StructArrayPublisher<Pose2d> visionPosePublisher = NetworkTableInstance
      .getDefault().getStructArrayTopic("visionPoses", Pose2d.struct).publish();

  private Supplier<Boolean> shouldSprint = () -> mainController.leftBumper().getAsBoolean();

  public RobotContainer() {
    shooterTracker = new TagTracking("limelight-shooter");
    backTracker = new TagTracking("limelight-back");
    swerveDrive = SwerveDriveFactory.createSwerveDrive(
        SwerveDriveFactory.SwerveImplementation.WPILIB,
        SwerveDriveFactory.RobotVariant.COMPETITION);

    shooterSubsystem = new ShooterSubsystem();
    transportSubsystem = new TransportSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    positioningCmd = new PositioningCmd(swerveDrive, shooterTracker, backTracker);

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

  private void registerCommand() {
    NamedCommands.registerCommand("deployIntake", intakeSubsystem.deployIntakeCmd());
    NamedCommands.registerCommand("intake", intakeSubsystem.intakeCmd());
    NamedCommands.registerCommand("previousShoot", shooterSubsystem.shootCmd());
    NamedCommands.registerCommand("shoot", new ShooterComboCmd(shooterSubsystem, transportSubsystem).withTimeout(5));

    NamedCommands.registerCommand("setLeftTrenchPose", Auto.setCurrentPoseCmd(AutoConstants.leftTrenchPose));
    NamedCommands.registerCommand("setMidPose", Auto.setCurrentPoseCmd(AutoConstants.midPostPose));
    NamedCommands.registerCommand("setRightTrenchPose", Auto.setCurrentPoseCmd(AutoConstants.rightTrenchPose));

    NamedCommands.registerCommand("toLeftTrenchPose", Auto.findToPoseCmd(AutoConstants.leftTrenchPose, 4));
    NamedCommands.registerCommand("toLeftNeutralPoseA", Auto.findToPoseCmd(AutoConstants.leftNeutralPoseA, 2));
    NamedCommands.registerCommand("toLeftNeutralPoseC", Auto.findToPoseCmd(AutoConstants.leftNeutralPoseC, 0, 2, 2));
    NamedCommands.registerCommand("toLeftShoot", Auto.findToPoseCmd(AutoConstants.leftShootPose, 0));
    NamedCommands.registerCommand("toRightTrenchPose", Auto.findToPoseCmd(AutoConstants.rightTrenchPose, 4));
    NamedCommands.registerCommand("toRightNeutralPoseA", Auto.findToPoseCmd(AutoConstants.rightNeutralPoseA, 2));
    NamedCommands.registerCommand("toRightNeutralPoseC", Auto.findToPoseCmd(AutoConstants.rightNeutralPoseC, 0, 2, 2));
    NamedCommands.registerCommand("toRightShoot", Auto.findToPoseCmd(AutoConstants.rightShootPose, 0));
    NamedCommands.registerCommand("toDepotPoseA", Auto.findToPoseCmd(AutoConstants.depotPoseA, 0));
    NamedCommands.registerCommand("toDepotPoseB", Auto.findToPoseCmd(AutoConstants.depotPoseB, 0));
    NamedCommands.registerCommand("toOutPostPose", Auto.findToPoseCmd(AutoConstants.outPostPose, 0));
  }

  private void configureBindings() {
    // position tracking
    positioningCmd.schedule();
    // swerve drive
    swerveDrive.setDefaultCommand(new SwerveControlCmd(swerveDrive, mainController, shouldSprint));
    mainController.start().onTrue(Commands.runOnce(() -> {
      swerveDrive.zeroGyro();
      swerveDrive.resetPose(new Pose2d(swerveDrive.getPose2d().getTranslation(), Rotation2d.fromDegrees(0)));
    }));
    // shooter
    mainController.rightBumper().whileTrue(new ShooterComboCmd(shooterSubsystem, transportSubsystem));
    mainController.x().toggleOnTrue(shooterSubsystem.shootCmd());
    // transport
    mainController.b().whileTrue(transportSubsystem.transportInCmd());
    // intake
    mainController.povUp().whileTrue(intakeSubsystem.deployIntakeCmd());
    mainController.povDown().whileTrue(intakeSubsystem.retractIntakeCmd());
    mainController.rightTrigger().whileTrue(intakeSubsystem.intakeCmd());
    mainController.leftTrigger().whileTrue(intakeSubsystem.reverseIntakeCmd());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}