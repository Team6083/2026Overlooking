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
  private final VictorSPX transportMotor = new VictorSPX(TransportConstants.transportMotorLowerID);

  public TransportSubsystem() {
    transportMotor.setInverted(TransportConstants.transportMotorInverted);
  }

  public void transportIn() {
    transportMotor.set(ControlMode.PercentOutput, TransportConstants.transportLowerMotorIn);
  }

  public void transportLoIn() {
    transportMotor.set(ControlMode.PercentOutput, TransportConstants.transportLowerMotorIn);
  }

  private void transportOut() {
    transportMotor.set(ControlMode.PercentOutput, TransportConstants.transportLowerMotorOut);
  }

  private void stopTransport() {
    transportMotor.set(ControlMode.PercentOutput, 0);
  }

  public Command transportInCmd() {
    Command cmd = runEnd(this::transportIn, this::stopTransport);
    cmd.setName("transportInCmd");
    return cmd;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("transport/motorOutputPercent", transportMotor.getMotorOutputPercent());
    SmartDashboard.putData("transport/subsystem", this);
    // This method will be called once per scheduler run
  }
}