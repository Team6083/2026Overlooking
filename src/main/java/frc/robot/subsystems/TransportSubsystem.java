// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransportConstant;

public class TransportSubsystem extends SubsystemBase {
  /** Creates a new transportSubsystem. */
  PWMVictorSPX tranpsortMotor = new PWMVictorSPX(TransportConstant.transportMotorID);

  public TransportSubsystem() {}

  private void transportIn() {
    tranpsortMotor.set(TransportConstant.transportMotorIn);
  }

  private void transportOut() {
    tranpsortMotor.set(TransportConstant.transportMotorOut);
  }

  private void stopTransport() {
    tranpsortMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}