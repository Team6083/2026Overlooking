package frc.robot.drivebase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule extends SubsystemBase {
    SparkMax turningMotor;
    SparkMax driveMotor;
    CANcoder canCoder;
    PIDController pid;
    double offset;

    private final double kMaxSpeedMetersPerSecond = 4.5;

    public SwerveModule(int turningMotorId, int driveMotorId, 
                        int canCoderId, double canCoderOffset, 
                        boolean turningInverted, boolean driveInverted){
        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        canCoder = new CANcoder(canCoderId);

        pid = new PIDController(0.5, 0, 0);
        pid.enableContinuousInput(-Math.PI, Math.PI);

        offset = canCoderOffset;
    }
    public double getAngleRadians() {
        double angle = canCoder.getAbsolutePosition()
                       .getValue()
                       .in(Units.Radians)
               - offset;
        return MathUtil.angleModulus(angle);
    }
    public void setDesiredState(SwerveModuleState desiredState) {
        Rotation2d currentAngle = Rotation2d.fromRadians(getAngleRadians());

        desiredState.optimize(currentAngle);
        SwerveModuleState optimized = desiredState;

        SmartDashboard.putNumber("optimize", desiredState.angle.getRadians());

        double turnOutput = pid.calculate(
            currentAngle.getRadians(),
            optimized.angle.getRadians()
        );
        turnOutput = MathUtil.clamp(turnOutput, -1.0, 1.0);
        turningMotor.set(turnOutput);

        double driveOutput = optimized.speedMetersPerSecond / kMaxSpeedMetersPerSecond;
        driveMotor.set(driveOutput);
    }

    public void setAngle(Rotation2d targetAngle) {
        double output = pid.calculate(
            getAngleRadians(),
            targetAngle.getRadians()
        );
        output = MathUtil.clamp(output, -1.0, 1.0);
        turningMotor.set(output);
    }

    public void stop() {
        turningMotor.set(0);
        driveMotor.set(0);
    }

}
