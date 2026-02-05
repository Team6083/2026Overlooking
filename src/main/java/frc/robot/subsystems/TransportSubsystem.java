// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;


public class TransportSubsystem extends SubsystemBase {
  /** Creates a new transportSubsystem. */
  VictorSP motorspeed = new VictorSP(0);

  public TransportSubsystem() {}

  private void transportIn() {
    motorspeed.set(0.5);
  }

  private void transportOut() {
    motorspeed.set(-0.5);
  }

  private void stopTransport() {
    motorspeed.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}