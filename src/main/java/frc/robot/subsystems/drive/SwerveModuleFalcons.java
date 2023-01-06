package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.PreferencesParser;
import frc.lib.drive.SwerveParameters;
import frc.lib.flowcontrol.EdgeTrigger;
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
public class SwerveModuleFalcons implements ISwerveModule {
  private final double kWheelDiameter; // meters, wheel is 4 inches in diameter
  private final int kEncoderResolution;
  private final double potOffsetRadians;
  private final double gearRatio;
  private boolean stopped = false;
  private double angleOnStop;
  private EdgeTrigger stoppedEdgeTrigger = new EdgeTrigger(false);

  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;
  private final CTREMagneticEncoder m_turningMagneticEncoder;

  private double desiredModuleSpeed;
  private double desiredModuleAngle;
  private String modulePosition;
  private double speedTicksPer100ms;
  private double speedRPM;
  private double desiredAngleDegrees;

  private static final double DEFAULT_DRIVE_P = 0.0004;
  private static final double DEFAULT_DRIVE_I = 0;
  private static final double DEFAULT_DRIVE_D = 0;
  private static final double DEFAULT_DRIVE_FF = 0.055;
  private static final double DEFAULT_TURN_P = 0.2;
  private static final double DEFAULT_TURN_I = 0;
  private static final double DEFAULT_TURN_D = 0;

  private static final double ENCODER_TICKS_PER_ROTATION = 2048;
  private static final double TURN_STEER_REDUCTION = 0.0978;
  private static final double DRIVE_LIMIT_CURRENT_THRESHOLD = 100; // 50
  private static final double DRIVE_LIMITED_CURRENT = 90; // 40
  private static final double TURN_LIMIT_CURRENT_THRESHOLD = 50;
  private static final double TURN_LIMITED_CURRENT = 40;
  private static final double LIMIT_CURRENT_AFTER = 0.1;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModuleFalcons(SwerveParameters parameters, Logger logger, PreferencesParser prefs) {
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
    potOffsetRadians = parameters.potOffset;
    this.kEncoderResolution = parameters.potResolution;
    m_turningMagneticEncoder =
        new CTREMagneticEncoder(
            parameters.turningEncoderChannel, kEncoderResolution, potOffsetRadians, false);
    this.gearRatio = parameters.driveGearRatio;
    kWheelDiameter = parameters.wheelDiameter;
    m_driveMotor.configClosedloopRamp(0.25);
    m_driveMotor.config_kP(
        0, prefs.tryGetValue(prefs::getDouble, "SwerveFalconsDriveP", DEFAULT_DRIVE_P));
    m_driveMotor.config_kI(
        0, prefs.tryGetValue(prefs::getDouble, "SwerveFalconsDriveI", DEFAULT_DRIVE_I));
    m_driveMotor.config_kD(
        0, prefs.tryGetValue(prefs::getDouble, "SwerveFalconsDriveD", DEFAULT_DRIVE_D));
    m_driveMotor.config_kF(
        0, prefs.tryGetValue(prefs::getDouble, "SwerveFalconsDriveFF", DEFAULT_DRIVE_FF));
    m_turningMotor.config_kP(
        0, prefs.tryGetValue(prefs::getDouble, "SwerveFalconsTurnP", DEFAULT_TURN_P));
    m_turningMotor.config_kI(
        0, prefs.tryGetValue(prefs::getDouble, "SwerveFalconsTurnI", DEFAULT_TURN_I));
    m_turningMotor.config_kD(
        0, prefs.tryGetValue(prefs::getDouble, "SwerveFalconsTurnD", DEFAULT_TURN_D));
    modulePosition = parameters.modulePosition.toString();
    zeroTurnMotors();

    /*
    The complicated trackable setup shown below is an example of the creation of a StringTrackable. To obtain each
    double in each StringTrackable, first a lambda is used the correct method/variable from the swerve module is called. Also, each double is separated by commas.

    The use of StringTrackables allows multiple doubles to be logged in a single file, which is useful when logging
    related pieces of data (such as a single module's desired angle and actual angle).
    */
    logger.addStringTrackable(
        () ->
            desiredModuleSpeed
                + ","
                + desiredAngleDegrees
                + ","
                + getSpeed()
                + ","
                + getAbsoluteAngleDegrees(),
        modulePosition + "_Module_State",
        3,
        "Desired Speed, Desired Angle, Actual Speed, Actual Angle");
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getSpeed(), new Rotation2d(getAbsoluteAngleRadians()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(getAbsoluteAngleRadians()));

    double targetAngle;
    double currentAngleRadians = getAngleRadians();

    // Calculate the drive output from the drive PID controller.
    speedRPM =
        (state.speedMetersPerSecond * 60 * gearRatio) / (Math.PI * kWheelDiameter); // m/s to rpm

    speedTicksPer100ms = (speedRPM * ENCODER_TICKS_PER_ROTATION) / (10 * 60);

    double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
    if (currentAngleRadiansMod < 0.0) {
      currentAngleRadiansMod += 2.0 * Math.PI;
    }

    if (!stopped) {
      targetAngle = state.angle.getRadians();
      stoppedEdgeTrigger.update(false);
    } else {
      targetAngle = angleOnStop;
      speedRPM = 0;
    }

    desiredModuleSpeed = (speedRPM * Math.PI * kWheelDiameter) / (60 * gearRatio); // rpm to m/s
    desiredModuleAngle = targetAngle;
    desiredAngleDegrees = state.angle.getDegrees();

    // Entering this we assume our target angle is in the domain [-2pi, 2pi]
    if (targetAngle < 0) {
      targetAngle += 2 * Math.PI;
    }
    // Now our target angle should be in the domain [0, 2pi)

    // targetAngle has the domain [0, 2pi) but the Falcon's encoder can go above that.
    // currentAngleRadians has an unbound domain
    // currentAngleRadiansMod is in the domain [0, 2pi)
    double adjustedReferenceAngleRadians =
        targetAngle + currentAngleRadians - currentAngleRadiansMod;
    if (targetAngle - currentAngleRadiansMod > Math.PI) {
      adjustedReferenceAngleRadians -= 2.0 * Math.PI;
    } else if (targetAngle - currentAngleRadiansMod < -Math.PI) {
      adjustedReferenceAngleRadians += 2.0 * Math.PI;
    }

    double targetAngleTicks =
        adjustedReferenceAngleRadians
            * ENCODER_TICKS_PER_ROTATION
            / (2 * Math.PI * TURN_STEER_REDUCTION);

    m_driveMotor.set(ControlMode.Velocity, speedTicksPer100ms);
    m_turningMotor.set(ControlMode.Position, targetAngleTicks);

    stopped = false;
  }

  /**
   * @return The analog input value of the encoder for the rotation motor
   */
  public double getAngleTicks() {
    return (m_turningMotor.getSelectedSensorPosition() * TURN_STEER_REDUCTION);
  }

  /**
   * @return The angle of the module's wheel in radians
   */
  public double getAngleRadians() {
    return getAngleTicks() * 2 * Math.PI / ENCODER_TICKS_PER_ROTATION;
  }

  /**
   * @return The angle of the module within the range [0, 2pi)
   */
  public double getAbsoluteAngleRadians() {
    return getAngleRadians() % (2.0 * Math.PI);
  }

  /**
   * @return The angle of the module's wheels in degrees
   */
  public double getAbsoluteAngleDegrees() {
    return (getAngleTicks() * 360 / ENCODER_TICKS_PER_ROTATION) % (360);
  }

  public double getDesiredAngle() {
    return desiredModuleAngle;
  }

  /**
   * @return The rpm of the drive motor
   */
  public double getSpeedNative() {
    return m_driveMotor.getSelectedSensorVelocity() * (60 * 10) / ENCODER_TICKS_PER_ROTATION;
  }

  public double getDesiredSpeed() {
    return desiredModuleSpeed;
  }

  /**
   * @return The speed of the module's wheel in meters/sec
   */
  public double getSpeed() {
    return (getSpeedNative() / (60 * gearRatio)) * Math.PI * kWheelDiameter;
  }

  /**
   * Updates Values on SmartDashboard. If these aren't showing up, double check that the call to
   * this method in SwerveDrive.java is uncommented.
   */
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

  /** Call this method to prevent the module from moving from its current position */
  public void halt() {
    stopped = true;
    if (stoppedEdgeTrigger.getRisingUpdate(stopped)) {
      // This makes sure that angleOnStop is in the range [0, 2pi)
      double angleOnStopMod = getAngleRadians() % (2.0 * Math.PI);
      if (angleOnStopMod < 0.0) {
        angleOnStopMod += 2.0 * Math.PI;
      }
      angleOnStop = angleOnStopMod;
    }
  }

  public void onStart() {
    zeroTurnMotors();
  }

  private void zeroTurnMotors() {
    double turningMotorOffset =
        m_turningMagneticEncoder.getAngleOffset()
            * ENCODER_TICKS_PER_ROTATION
            / (2 * Math.PI * TURN_STEER_REDUCTION);
    m_turningMotor.setSelectedSensorPosition(turningMotorOffset);
  }
}
