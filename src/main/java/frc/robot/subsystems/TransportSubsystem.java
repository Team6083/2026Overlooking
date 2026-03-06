// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransportConstants;

public class TransportSubsystem extends SubsystemBase {
  /** Creates a new transportSubsystem. */
  private final VictorSPX transportUpperMotor = new VictorSPX(TransportConstants.transportMotorUpperID);
  private final VictorSPX transportLowerMotor = new VictorSPX(TransportConstants.transportMotorLowerID);

  public TransportSubsystem() {
    transportUpperMotor.setInverted(TransportConstants.transportUpperMotorInverted);
    transportLowerMotor.setInverted(TransportConstants.transportLowerMotorInverted);
  }

  public void transportIn() {
    transportUpperMotor.set(ControlMode.PercentOutput, TransportConstants.transportUpperMotorIn);
    transportLowerMotor.set(ControlMode.PercentOutput, TransportConstants.transportLowerMotorIn);
  }

  private void transportOut() {
    transportUpperMotor.set(ControlMode.PercentOutput, TransportConstants.transportUpperMotorOut);
    transportLowerMotor.set(ControlMode.PercentOutput, TransportConstants.transportLowerMotorOut);
  }

  private void stopTransport() {
    transportUpperMotor.set(ControlMode.PercentOutput, 0);
    transportLowerMotor.set(ControlMode.PercentOutput, 0);
  }

  public Command transportInCmd() {
    Command cmd = runEnd(this::transportIn, this::stopTransport);
    cmd.setName("transportInCmd");
    return cmd;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("transportUpperMotorSpeed", transportUpperMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("transportLowerMotorSpeed", transportLowerMotor.getMotorOutputPercent());
    SmartDashboard.putData("TransportSubsystem", this);
    // This method will be called once per scheduler run
  }
}