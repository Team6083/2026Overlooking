// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkMax climbMotor = new SparkMax(ClimberConstants.climbMotorId, MotorType.kBrushless);

  private final Encoder encoder = new Encoder(ClimberConstants.climbEncoderIdA, ClimberConstants.climbEncoderIdB);

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    SparkMaxConfig climberMotorConfig = new SparkMaxConfig();
    climberMotorConfig
        .inverted(ClimberConstants.climbMotorInverted)
        .idleMode(IdleMode.kBrake);
    climbMotor.configure(climberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void climbUp() {
    climbMotor.set(ClimberConstants.climbUpSpeed);
  }

  private void climbDown() {
    climbMotor.set(ClimberConstants.climbDownSpeed);
  }

  private void stopClimb() {
    climbMotor.set(0);
  }

  private double getClimbPosition() {
    return encoder.getDistance(); // Adjust distance per pulse if necessary
  }

  public Command climbUpCmd() {
    Command cmd = runEnd(this::climbUp, this::stopClimb);
    cmd.setName("climbUpCmd");
    return cmd;
  }

  public Command climbDownCmd() {
    Command cmd = runEnd(this::climbDown, this::stopClimb);
    cmd.setName("climbDownCmd");
    return cmd;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("climber/climberPosition", getClimbPosition());
    SmartDashboard.putNumber("climber/climberOutput", climbMotor.get());
    SmartDashboard.putData("climber/subsystem", this);
  }
}
