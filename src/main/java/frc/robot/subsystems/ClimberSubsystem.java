// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkMax climbMotor = new SparkMax(ClimberConstants.climbMotorId, MotorType.kBrushless);

  private final Encoder encoder = new Encoder(ClimberConstants.climbEncoderIdA, ClimberConstants.climbEncoderIdB);
  private final PIDController pidClimberController = new PIDController(ClimberConstants.climberKp, ClimberConstants.climberKi,
        ClimberConstants.climberKd);

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
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

  private double pidClimbControl(double setpoint) {
    double output = pidClimberController.calculate(getClimbPosition(), setpoint);
    climbMotor.set(output);
    return output;
  }

  public Command climbUpCmd() {
    Command cmd = runEnd(this::climbUp, this::stopClimb);
    cmd.setName("climb Up");
    return cmd;
  }

  public Command climbDownCmd() {
    Command cmd = runEnd(this::climbDown, this::stopClimb);
    cmd.setName("climb Down");
    return cmd;
  }
  
  public Command manualClimbCmd(double setpoint) {
    Command cmd = run(() -> pidClimbControl(setpoint));
    cmd.setName("manual Climb Control");
    return cmd;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ClimberPosition", getClimbPosition());
    SmartDashboard.putNumber("ClimberOutput", climbMotor.get());
    SmartDashboard.putData("ClimberSubsystem", this);
  }
}
