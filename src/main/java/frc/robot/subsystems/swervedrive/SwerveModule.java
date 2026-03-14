package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Minutes;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstant;
import frc.robot.Constants.SwerveModuleConstant;

public class SwerveModule extends SubsystemBase {
  private final SparkMax turningMotor;
  private final SparkMax driveMotor;
  private final CANcoder turningEncoder;
  private final PIDController rotPIDController;
  private final RelativeEncoder driveEncoder;

  private double turningMotorVoltage;
  private double driveMotorVoltage;

  private final Alert activeFaultAlert = new Alert("rev active faults!", AlertType.kError);
  private final Alert stickyFaultAlert = new Alert("rev sticky faults!", AlertType.kWarning);

  public SwerveModule(SwerveModuleConstant swerveModuleConstant) {
    turningMotor = new SparkMax(swerveModuleConstant.turningMotorId(), MotorType.kBrushless);
    SparkMaxConfig turningMotorConfig = new SparkMaxConfig();
    turningMotorConfig.smartCurrentLimit(15)
        .idleMode(IdleMode.kCoast)
        .inverted(swerveModuleConstant.turningInverted());
    turningMotor.configure(turningMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveMotor = new SparkMax(swerveModuleConstant.driveMotorId(), MotorType.kBrushless);
    SparkMaxConfig driveMotorConfig = new SparkMaxConfig();
    driveMotorConfig.smartCurrentLimit(40)
        .idleMode(IdleMode.kBrake)
        .inverted(swerveModuleConstant.driveInverted());
    driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveEncoder = driveMotor.getEncoder();

    turningEncoder = new CANcoder(swerveModuleConstant.canCoderId());
    CANcoderConfiguration turningEncoderConfiguration = new CANcoderConfiguration();
    turningEncoderConfiguration.MagnetSensor.MagnetOffset = swerveModuleConstant.canCoderOffset();
    turningEncoder.getConfigurator().apply(turningEncoderConfiguration);

    rotPIDController = new PIDController(0.225, 0, 0);
    rotPIDController.enableContinuousInput(-Math.PI, Math.PI);

    driveMotorVoltage = 0;
    turningMotorVoltage = 0;

    setName(swerveModuleConstant.name() + "Module");
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

  // to the get the position by wpi function
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDriveDistance(), getRotation2d());
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

    double turnOutput = rotPIDController.calculate(
        currentAngle.getRadians(),
        optimized.angle.getRadians());
    turnOutput = MathUtil.clamp(turnOutput, -1.0, 1.0);
    turningMotor.set(turnOutput);

    double driveOutput = optimized.speedMetersPerSecond
        / ModuleConstant.kMaxModuleSpeed.in(MetersPerSecond);
    driveMotor.set(driveOutput);

    driveMotorVoltage = driveOutput * driveMotor.getBusVoltage();
    turningMotorVoltage = turnOutput * turningMotor.getBusVoltage();
  }

  public void setAngle(Rotation2d targetAngle) {
    double output = rotPIDController.calculate(
        getAngleRadians(),
        targetAngle.getRadians());
    output = MathUtil.clamp(output, -1.0, 1.0);

    turningMotorVoltage = output * turningMotor.getBusVoltage();
    turningMotor.set(output);
  }

  public void stop() {
    turningMotor.set(0);
    driveMotor.set(0);

    turningMotorVoltage = 0;
    driveMotorVoltage = 0;
  }

  private void setFaultAlerts() {
    var driveFaults = driveMotor.getFaults().toString();
    var turningFaults = turningMotor.getFaults().toString();

    var driveStickyFaults = driveMotor.getStickyFaults().toString();
    var turningStickyFaults = turningMotor.getStickyFaults().toString();

    activeFaultAlert.set(driveMotor.hasActiveFault() || turningMotor.hasActiveFault());
    activeFaultAlert.setText(this.getName() + "Drive" + driveFaults +
        " Turning " + turningFaults);

    stickyFaultAlert.set(driveMotor.hasStickyFault() || turningMotor.hasStickyFault());
    stickyFaultAlert.setText(this.getName() + "Drive" + driveStickyFaults +
        " Turning " + turningStickyFaults);
  }

  @Override
  public void periodic() {
    String prefix = "drive/" + this.getName() + "/";
    SmartDashboard.putData(prefix + "anglePID", rotPIDController);
    SmartDashboard.putNumber(prefix + "turningMotorOutput", turningMotor.get());
    SmartDashboard.putNumber(prefix + "angleRadians", getAngleRadians());
    SmartDashboard.putNumber(prefix + "driveRateMps", getDriveRate().in(MetersPerSecond));
    SmartDashboard.putNumber(prefix + "driveDistanceM", getDriveDistance().in(Meters));
    SmartDashboard.putNumber(prefix + "driveMotorVoltage", driveMotorVoltage);
    SmartDashboard.putNumber(prefix + "driveMotorAppliedVoltage",
        driveMotor.getAppliedOutput() * driveMotor.getBusVoltage());
    SmartDashboard.putNumber(prefix + "driveMotorCurrentAmps", driveMotor.getOutputCurrent());
    SmartDashboard.putNumber(prefix + "turningMotorVoltage", turningMotorVoltage);
    SmartDashboard.putNumber(prefix + "turningMotorAppliedVoltage",
        turningMotor.getAppliedOutput() * turningMotor.getBusVoltage());
    SmartDashboard.putNumber(prefix + "turningMotorCurrentAmps", turningMotor.getOutputCurrent());

    setFaultAlerts();
  }
}
