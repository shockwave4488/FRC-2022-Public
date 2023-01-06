package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.PreferencesParser;
import frc.lib.controlsystems.SimPID;
import frc.lib.drive.SwerveParameters;
import frc.lib.logging.Logger;
import frc.lib.sensors.Potentiometer;

/** A SwerveModule class that operates Spark Neos */
public class SwerveModuleNeos implements ISwerveModule {
  private final double kWheelDiameter; // meters, wheel is 4 inches in diameter
  private final int kEncoderResolution;
  private final double potOffset;
  private final double gearRatio;
  private boolean stopped = false;

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final Potentiometer m_turningEncoder;

  private SparkMaxPIDController m_drivePIDController;

  private final SimPID m_turningPIDController;

  private double desiredModuleSpeed;
  private double desiredModuleAngle;
  private String modulePosition;
  private final Translation2d moduleLocation;

  private static final double DEFAULT_DRIVE_P = 0.00005;
  private static final double DEFAULT_DRIVE_I = 0;
  private static final double DEFAULT_DRIVE_D = 0;
  private static final double DEFAULT_DRIVE_FF = 0.00016;
  private static final double DEFAULT_TURN_P = 0.0006;
  private static final double DEFAULT_TURN_I = 0;
  private static final double DEFAULT_TURN_D = 0.0001;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModuleNeos(SwerveParameters parameters, Logger logger, PreferencesParser prefs) {
    m_driveMotor = new CANSparkMax(parameters.driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(parameters.turningMotorChannel, MotorType.kBrushless);
    m_turningEncoder = new Potentiometer(parameters.turningEncoderChannel);
    this.potOffset = parameters.potOffset;
    this.gearRatio = parameters.driveGearRatio;
    kEncoderResolution = parameters.potResolution;
    kWheelDiameter = parameters.wheelDiameter;
    m_driveMotor.setClosedLoopRampRate(1);
    m_drivePIDController = m_driveMotor.getPIDController();
    m_drivePIDController.setP(prefs.tryGetDouble("SwerveNeosDriveP", DEFAULT_DRIVE_P));
    m_drivePIDController.setI(prefs.tryGetDouble("SwerveNeosDriveI", DEFAULT_DRIVE_I));
    m_drivePIDController.setD(prefs.tryGetDouble("SwerveNeosDriveD", DEFAULT_DRIVE_D));
    m_drivePIDController.setFF(prefs.tryGetDouble("SwerveNeosDriveFF", DEFAULT_DRIVE_FF));
    m_turningPIDController =
        new SimPID(
            prefs.tryGetDouble("SwerveNeosTurnP", DEFAULT_TURN_P),
            prefs.tryGetDouble("SwerveNeosTurnI", DEFAULT_TURN_I),
            prefs.tryGetDouble("SwerveNeosTurnD", DEFAULT_TURN_D));
    m_turningPIDController.setWrapAround(0, 4096);
    modulePosition = parameters.modulePosition.toString();
    moduleLocation = new Translation2d(parameters.moduleX, parameters.moduleY);

    logger.addStringTrackable(
        () ->
            desiredModuleSpeed
                + ","
                + desiredModuleAngle
                + ","
                + getSpeed()
                + ","
                + getAbsoluteAngleDegrees(),
        modulePosition + " Module State",
        10,
        "Desired Speed, Desired Angle, Actual Speed, Actual Angle");
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getSpeed(), new Rotation2d(getAngleRadians()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(getAngleRadians()));
    double targetAngle;

    // Calculate the drive output from the drive PID controller.
    double speedRPM =
        (state.speedMetersPerSecond * 60 * gearRatio) / (Math.PI * kWheelDiameter); // m/s to rpm
    // speedRPM *= reversed ? -1 : 1;

    if (!stopped) {
      targetAngle = state.angle.getRadians() / (2 * Math.PI) * kEncoderResolution;
    } else {
      targetAngle = getAngleTicks();
      speedRPM = 0;
    }

    desiredModuleSpeed = (speedRPM * Math.PI * kWheelDiameter) / (60 * gearRatio);
    desiredModuleAngle = targetAngle * 360 / kEncoderResolution;

    m_turningPIDController.setDesiredValue(targetAngle);
    double power = m_turningPIDController.calcPID(getAngleTicks());

    m_drivePIDController.setReference(speedRPM, CANSparkMax.ControlType.kVelocity);
    m_turningMotor.set(power);

    stopped = false;
  }

  /** @return The analog input value of the encoder for the rotation motor */
  public double getAngleAnalog() {
    return m_turningEncoder.get();
  }

  /**
   * @return The value of the rotation encoder, adjusted for the potentiometer offset. Still returns
   *     values between 0 and the encoder resolution.
   */
  public double getAngleTicks() {
    return kEncoderResolution
        - ((m_turningEncoder.get() - potOffset + kEncoderResolution) % kEncoderResolution);
  }

  /** @return The angle of the module's wheel in radians */
  public double getAngleRadians() {
    return (getAngleTicks() * 2 * Math.PI / kEncoderResolution);
  }

  /** @return The angle of the module's wheels in degrees */
  public double getAbsoluteAngleDegrees() {
    return (getAngleTicks() * 360 / kEncoderResolution);
  }

  public double getDesiredAngle() {
    return desiredModuleAngle;
  }

  /** @return The rpm of the drive motor */
  public double getSpeedNative() {
    return m_driveMotor.getEncoder().getVelocity(); // get speed from spark
  }

  public double getDesiredSpeed() {
    return desiredModuleSpeed;
  }

  /** @return The speed of the module's wheel in meters/sec */
  public double getSpeed() {
    return (getSpeedNative() / (60 * gearRatio)) * Math.PI * kWheelDiameter;
  }

  public String getRobotPosition() {
    return modulePosition;
  }

  public Translation2d getLocation() {
    return moduleLocation;
  }

  /** Call this method to prevent the module from moving from its current position */
  public void halt() {
    stopped = true;
  }

  @Override
  public void onStart() {}

  @Override
  public void updateSmartDashboard() {
    /*
    This is commented so we can quickly access this code if we need to debug our swerve modules,
    but don't have to devote the CPU cycles to always have this running.
    */
    // SmartDashboard.putNumber(modulePosition + " Desired Speed", getDesiredSpeed());
    // SmartDashboard.putNumber(modulePosition + " Desired Angle", getDesiredAngle());
    // SmartDashboard.putNumber(modulePosition + " Actual Speed", getSpeed());
    // SmartDashboard.putNumber(modulePosition + " Actual Angle", getAbsoluteAngleDegrees());
    // SmartDashboard.putNumber(modulePosition + " Angle Ticks", getAngleTicks());
  }
}
