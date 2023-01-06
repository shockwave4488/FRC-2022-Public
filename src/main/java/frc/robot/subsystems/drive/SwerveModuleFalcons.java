package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.PreferencesParser;
import frc.lib.drive.SwerveParameters;
import frc.lib.logging.Logger;
import frc.lib.sensors.CTREMagneticEncoder;

/**
 * A SwerveModule class that operates Falcon 500s.
 *
 * <p>Some of the logic in this class was used from Team 2910 Jack In The Bot's
 * (https://github.com/FRCTeam2910/2021CompetitionRobot/blob/master/src/main/java/org/frcteam2910/c2020/subsystems/DrivetrainSubsystem.java)
 * and Swerve Drive Specialties'
 * (https://github.com/SwerveDriveSpecialties/swerve-lib/tree/55f3f1ad9e6bd81e56779d022a40917aacf8d3b3/src/main/java/com/swervedrivespecialties/swervelib)
 * open source swerve code.
 */
public class SwerveModuleFalcons extends SwerveModule {
  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;
  private final CTREMagneticEncoder m_turningMagneticEncoder;

  private static final double DEFAULT_DRIVE_P = 0.0004;
  private static final double DEFAULT_DRIVE_I = 0;
  private static final double DEFAULT_DRIVE_D = 0;
  private static final double DEFAULT_DRIVE_FF = 0.055;
  private static final double DEFAULT_TURN_P = 0.2;
  private static final double DEFAULT_TURN_I = 0;
  private static final double DEFAULT_TURN_D = 0;

  private static final double TURN_STEER_REDUCTION = 0.0978;
  private static final double DRIVE_LIMIT_CURRENT_THRESHOLD = 100; // 50
  private static final double DRIVE_LIMITED_CURRENT = 90; // 40
  private static final double TURN_LIMIT_CURRENT_THRESHOLD = 50;
  private static final double TURN_LIMITED_CURRENT = 40;
  private static final double LIMIT_CURRENT_AFTER = 0.1;

  /**
   * Constructs a SwerveModuleFalcon.
   *
   * @param parameters Module-specific parameters
   */
  public SwerveModuleFalcons(SwerveParameters parameters, Logger logger, PreferencesParser prefs) {
    super(parameters, logger, parameters.relativeTurningEncoderResolution);
    m_driveMotor = new TalonFX(parameters.driveMotorChannel);
    m_driveMotor.configFactoryDefault();
    m_turningMotor = new TalonFX(parameters.turningMotorChannel);
    m_turningMotor.configFactoryDefault();
    m_driveMotor.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(
            true, DRIVE_LIMITED_CURRENT, DRIVE_LIMIT_CURRENT_THRESHOLD, LIMIT_CURRENT_AFTER));
    m_turningMotor.setInverted(true);
    m_turningMotor.setNeutralMode(NeutralMode.Brake);
    m_turningMotor.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(
            true, TURN_LIMITED_CURRENT, TURN_LIMIT_CURRENT_THRESHOLD, LIMIT_CURRENT_AFTER));
    m_turningMagneticEncoder =
        new CTREMagneticEncoder(
            parameters.turningEncoderChannel,
            parameters.absoluteEncoderResolution,
            absoluteEncoderOffset,
            false);
    m_driveMotor.configClosedloopRamp(0.25);
    m_driveMotor.config_kP(0, prefs.tryGetDouble("SwerveFalconsDriveP", DEFAULT_DRIVE_P));
    m_driveMotor.config_kI(0, prefs.tryGetDouble("SwerveFalconsDriveI", DEFAULT_DRIVE_I));
    m_driveMotor.config_kD(0, prefs.tryGetDouble("SwerveFalconsDriveD", DEFAULT_DRIVE_D));
    m_driveMotor.config_kF(0, prefs.tryGetDouble("SwerveFalconsDriveFF", DEFAULT_DRIVE_FF));
    m_turningMotor.config_kP(0, prefs.tryGetDouble("SwerveFalconsTurnP", DEFAULT_TURN_P));
    m_turningMotor.config_kI(0, prefs.tryGetDouble("SwerveFalconsTurnI", DEFAULT_TURN_I));
    m_turningMotor.config_kD(0, prefs.tryGetDouble("SwerveFalconsTurnD", DEFAULT_TURN_D));
    zeroTurnMotors();
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    double currentAngleRadians = getAngleRadians();
    setDesiredValues(desiredState, currentAngleRadians);

    // Calculate the drive output from the drive PID controller.
    double speedRPM = metersPerSecToRPM(desiredModuleSpeed);
    double speedTicksPer100ms = (speedRPM * turningEncoderResolution) / (10 * 60);

    double currentAngleRadiansMod = MathUtil.angleModulus(currentAngleRadians); // domain (-pi, pi]

    // Get target angle by adding the delta of the desired bounded angle and actual bounded angle to
    // the current unbounded angle, and converting to ticks.
    double targetAngleTicks =
        (currentAngleRadians + desiredModuleAngle - currentAngleRadiansMod)
            * turningEncoderResolution
            / (2 * Math.PI * TURN_STEER_REDUCTION);

    m_driveMotor.set(ControlMode.Velocity, speedTicksPer100ms);
    m_turningMotor.set(ControlMode.Position, targetAngleTicks);
  }

  @Override
  protected double getDriveRotations() {
    return m_driveMotor.getSelectedSensorPosition() / driveEncoderResolution;
  }

  /**
   * Note, this method returns position of the integrated TalonFX encoder rather than the turning
   * mag encoder, because the integrated encoder is used for the actual PID control.
   *
   * @return The digital position value of the encoder for the rotation motor.
   */
  @Override
  protected double getAngleTicks() {
    return (m_turningMotor.getSelectedSensorPosition() * TURN_STEER_REDUCTION);
  }

  @Override
  protected double getSpeedNative() {
    return m_driveMotor.getSelectedSensorVelocity() * (60 * 10) / turningEncoderResolution;
  }

  @Override
  public void updateSmartDashboard() {
    /*
    This is commented so we can quickly access this code if we need to debug our swerve modules,
    but don't have to devote the CPU cycles to always have this running.
    */
    // SmartDashboard.putNumber(modulePosition + " Drive Desired Speed", getDesiredSpeed());
    /*
    SmartDashboard.putNumber(
        modulePosition + " Desired Angle Degrees", getDesiredAngle() * 180 / Math.PI);
    */
    // SmartDashboard.putNumber(modulePosition + " Drive Actual Speed", getSpeed());
    // SmartDashboard.putNumber(
    //    modulePosition + " Actual Ticks Angle", m_turningMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber(modulePosition + " Actual Degrees Angle", getAbsoluteAngleDegrees());
    SmartDashboard.putNumber(
        modulePosition + " Mag Angle", m_turningMagneticEncoder.getAngleOffset());
    /*
    SmartDashboard.putNumber(
        modulePosition + " Turn Motor Velocity",
        Math.abs(m_turningMotor.getSelectedSensorVelocity()));
    */
    // SmartDashboard.putNumber(modulePosition + " power", m_turningMotor.getMotorOutputPercent());
  }

  @Override
  public void onStart() {
    zeroTurnMotors();
  }

  private void zeroTurnMotors() {
    double turningMotorOffset =
        m_turningMagneticEncoder.getAngleOffset() // radians
            * turningEncoderResolution
            / (2 * Math.PI * TURN_STEER_REDUCTION);
    m_turningMotor.setSelectedSensorPosition(turningMotorOffset);
  }
}
