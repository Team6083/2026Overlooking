// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class IntakeSubsystem extends SubsystemBase {
    SparkMax intakeMotor = new SparkMax(10, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    Joystick driverJoystick = new Joystick(0);

  public IntakeSubsystem() {
    config.inverted(false)
          .smartCurrentLimit(30)
          .idleMode(IdleMode.kBrake);
    intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void intake() {
  
  }

  private void reverseIntake() {
    intakeMotor.set(-0.5);
  }

  private void stopIntake() {
    intakeMotor.set(0);
  }

  private void rotateUp() {
   intakeMotor.set(0.5);
  }

  private void rotateDown() {
   intakeMotor.set(-0.5);
  }

  private void stopRotate() {
    intakeMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (driverJoystick.getRawButton(2)) {
            rotateUp();
        } 
        else if (driverJoystick.getRawButton(3)) {
            rotateDown();
        } 
        else {
            stopRotate();
            
    }

  }
}
