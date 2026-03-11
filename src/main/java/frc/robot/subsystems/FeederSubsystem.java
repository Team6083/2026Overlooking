// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FeederSubsystem extends SubsystemBase {
  /** Creates a new FeederSubsystem. */
  private final VictorSPX feederMotor = new VictorSPX(FeederConstants.feederMotorID);

  public FeederSubsystem() {
    feederMotor.setInverted(FeederConstants.feederMotorInverted);
  }

  public void feedIn() {
    feederMotor.set(ControlMode.PercentOutput, FeederConstants.feederMotorIn);
  }

  public void feedOut() {
    feederMotor.set(ControlMode.PercentOutput, FeederConstants.feederMotorOut);
  }

  public void feedStop() {
    feederMotor.set(ControlMode.PercentOutput, 0);
  }

  public Command feedInCmd() {
    Command cmd = runEnd(this::feedIn, this::feedStop);
    cmd.setName("feedInCmd");
    return cmd;
  } 

  @Override
  public void periodic() {
    SmartDashboard.putNumber("feeder/motorOutputPercent", feederMotor.getMotorOutputPercent());
    // This method will be called once per scheduler run
  }
}
