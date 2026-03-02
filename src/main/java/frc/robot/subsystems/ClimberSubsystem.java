// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkMax climbMotor;

  private final Encoder encoder;
  private final PIDController pidController;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    climbMotor = new SparkMax(ClimberConstants.climbMotorId, MotorType.kBrushless);

    encoder = new Encoder(ClimberConstants.climbEncoderIdA, ClimberConstants.climbEncoderIdB);
    pidController = new PIDController(ClimberConstants.climberKp, ClimberConstants.climberKi,
        ClimberConstants.climberKd); // Adjust PID constants as needed
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

  private double manualClimbControl(double setpoint) {
    double output = pidController.calculate(getClimbPosition(), setpoint);
    climbMotor.set(output);
    return output;
  }

  public Command climbUpCmd() {
    Command cmd = runEnd(this::climbUp, this::stopClimb);
    cmd.setName("Climb Up");
    return cmd;
  }

  public Command climbDownCmd() {
    Command cmd = runEnd(this::climbDown, this::stopClimb);
    cmd.setName("Climb Down");
    return cmd;
  }
  
  public Command manualClimbCmd(double setpoint) {
    Command cmd = run(() -> manualClimbControl(setpoint));
    cmd.setName("Manual Climb Control");
    return cmd;
  }
  public Command stopClimbCmd() {
    Command cmd = runOnce(this::stopClimb);
    cmd.setName("Stop Climb");
    return cmd;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
