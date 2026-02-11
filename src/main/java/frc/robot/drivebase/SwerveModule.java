package frc.robot.drivebase;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Minutes;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstant;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule extends SubsystemBase {
  private final SparkMax turningMotor;
  private final SparkMax driveMotor;
  private final CANcoder turningEncoder;
  private final PIDController rotPIDController;
  private final RelativeEncoder driveEncoder;

  public SwerveModule(int turningMotorId, int driveMotorId,
      int canCoderId, double canCoderOffset,
      boolean turningInverted, boolean driveInverted, String name) {
    turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
    SparkMaxConfig turningMotorConfig = new SparkMaxConfig();
    turningMotorConfig.smartCurrentLimit(40)
        .idleMode(IdleMode.kCoast)
        .inverted(turningInverted);
    turningMotor.configure(turningMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
    SparkMaxConfig driveMotorConfig = new SparkMaxConfig();
    driveMotorConfig.smartCurrentLimit(40)
        .idleMode(IdleMode.kBrake)
        .inverted(driveInverted);
    driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    driveEncoder = driveMotor.getEncoder();

    turningEncoder = new CANcoder(canCoderId);
    CANcoderConfiguration turningEncoderConfiguration = new CANcoderConfiguration();
    turningEncoderConfiguration.MagnetSensor.MagnetOffset = canCoderOffset;
    turningEncoder.getConfigurator().apply(turningEncoderConfiguration);

    rotPIDController = new PIDController(0.5, 0, 0);
    rotPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public double getAngleRadians() {
    double angle = turningEncoder.getAbsolutePosition()
        .getValue()
        .in(Units.Radians);
    return MathUtil.angleModulus(angle);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveRate().in(MetersPerSecond),
        Rotation2d.fromRadians(getAngleRadians()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveMotor.getEncoder().getPosition() / 6.75 * 2.0 * Math.PI * 0.0508,
        Rotation2d.fromRadians(getAngleRadians()));
  }

  // to get the drive distance
  public Distance getDriveDistance() {
    return ModuleConstant.kWheelRadius.times(2.0 * Math.PI)
        .times(driveEncoder.getPosition() / 6.75);
  }

  // calculate the rate of the drive
  public LinearVelocity getDriveRate() {
    return Meters.per(Minutes).of(driveEncoder.getVelocity() / 6.75 * 2.0 * Math.PI
        * ModuleConstant.kWheelRadius.in(Meters));
  }

  // to get rotation of turning motor
  public Rotation2d getRotation2d() {
    return new Rotation2d(
        Math.toRadians(
            turningEncoder.getAbsolutePosition().getValueAsDouble() * 360.0));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    Rotation2d currentAngle = Rotation2d.fromRadians(getAngleRadians());

    desiredState.optimize(currentAngle);
    SwerveModuleState optimized = desiredState;

    SmartDashboard.putNumber("optimize", desiredState.angle.getRadians());

    double turnOutput = rotPIDController.calculate(
        currentAngle.getRadians(),
        optimized.angle.getRadians());
    turnOutput = MathUtil.clamp(turnOutput, -1.0, 1.0);
    turningMotor.set(turnOutput);

    double driveOutput = optimized.speedMetersPerSecond / 4;
    driveMotor.set(driveOutput);
  }

  public void setAngle(Rotation2d targetAngle) {
    double output = rotPIDController.calculate(
        getAngleRadians(),
        targetAngle.getRadians());
    output = MathUtil.clamp(output, -1.0, 1.0);
    turningMotor.set(output);
  }

  public void stop() {
    turningMotor.set(0);
    driveMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putData(this.getName() + "AnglePID", rotPIDController);
    SmartDashboard.putNumber(this.getName() + "MotorOutput", turningMotor.get());
    SmartDashboard.putNumber(this.getName() + "AngleRadius", getAngleRadians());
  }
}
