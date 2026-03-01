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
  VictorSPX transportLeftMotor = new VictorSPX(TransportConstants.transportMotorLeftID);
  VictorSPX transportRightMotor = new VictorSPX(TransportConstants.transportMotorRightID);

  public TransportSubsystem() {
    transportLeftMotor.setInverted(true);
    transportRightMotor.setInverted(true);
  }

  public void transportIn() {
    transportLeftMotor.set(ControlMode.PercentOutput, TransportConstants.transportMotorIn);
    transportRightMotor.set(ControlMode.PercentOutput, TransportConstants.transportMotorIn);
  }

  private void transportOut() {
    transportLeftMotor.set(ControlMode.PercentOutput, TransportConstants.transportMotorOut);
    transportRightMotor.set(ControlMode.PercentOutput, TransportConstants.transportMotorOut);
  }

  private void stopTransport() {
    transportLeftMotor.set(ControlMode.PercentOutput, 0);
    transportRightMotor.set(ControlMode.PercentOutput, 0);
  }

  public Command transportInCmd() {
    Command cmd = runEnd(this::transportIn, this::stopTransport);
    cmd.setName("transportInCmd");
    return cmd;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("transportLeftMotorSpeed", transportLeftMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("transportRightMotorSpeed", transportRightMotor.getMotorOutputPercent());
    SmartDashboard.putData("TransportSubsystem", this);
    // This method will be called once per scheduler run
  }
}