// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransportConstants;

public class TransportSubsystem extends SubsystemBase {
  /** Creates a new transportSubsystem. */
  PWMVictorSPX transportMotor = new PWMVictorSPX(TransportConstants.transportMotorID);

  public TransportSubsystem() {}

  private void transportIn() {
    transportMotor.set(TransportConstants.transportMotorIn);
  }

  private void transportOut() {
    transportMotor.set(TransportConstants.transportMotorOut);
  }

  private void stopTransport() {
    transportMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}