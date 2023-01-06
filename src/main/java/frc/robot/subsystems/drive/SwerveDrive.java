package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.logging.Logger;
import frc.lib.sensors.NavX;
import frc.lib.wpiextensions.ShockwaveSubsystemBase;

/** Represents a swerve drive style drivetrain. */
public class SwerveDrive extends ShockwaveSubsystemBase {
  public static final double kMaxSpeed = 3.5; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // rotations per second

  private static final double kTrackLength = 0.5715; // front to back
  private static final double kTrackWidth = 0.4826; // left to right

  private final Translation2d m_frontLeftLocation =
      new Translation2d(kTrackLength / 2, kTrackWidth / 2);
  private final Translation2d m_frontRightLocation =
      new Translation2d(kTrackLength / 2, -kTrackWidth / 2);
  private final Translation2d m_backLeftLocation =
      new Translation2d(-kTrackLength / 2, kTrackWidth / 2);
  private final Translation2d m_backRightLocation =
      new Translation2d(-kTrackLength / 2, -kTrackWidth / 2);

  private final ISwerveModule m_frontLeft;
  private final ISwerveModule m_frontRight;
  private final ISwerveModule m_backLeft;
  private final ISwerveModule m_backRight;

  private final NavX m_gyro;
  private final SwerveDriveKinematics m_kinematics;
  private final SwerveDriveOdometry m_odometry;
  private final Logger logger;

  private Pose2d currentPose;
  private Field2d currentFieldPos = new Field2d();
  private double fieldPoseX;
  private double fieldPoseY;

  private double visionEstimatedX = 0;
  private double visionEstimatedY = 0;

  /**
   * The drive class for swerve robots
   *
   * @param gyro A NavX that's used to get the angle of the robot
   * @param modules Array of swerve modules in the order: front left, front right, back left, back
   *     right
   */
  public SwerveDrive(NavX gyro, ISwerveModule[] modules, Logger logger) {
    m_gyro = gyro;
    this.logger = logger;
    m_frontLeft = modules[0];
    m_frontRight = modules[1];
    m_backLeft = modules[2];
    m_backRight = modules[3];
    m_kinematics =
        new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyro.getYaw(), getModulePositions());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getYaw())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    setModuleStates(swerveModuleStates);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(m_gyro.getYaw(), getModulePositions());
  }

  @Override
  public void onStart() {
    m_frontLeft.onStart();
    m_frontRight.onStart();
    m_backLeft.onStart();
    m_backRight.onStart();
  }

  @Override
  public void onStop() {}

  @Override
  public void zeroSensors() {
    m_gyro.reset();
  }

  @Override
  public void updateSmartDashboard() {
    // SmartDashboard updates are commeneted out because they are performance heavy and cause loop
    // overruns
    m_frontLeft.updateSmartDashboard();
    m_frontRight.updateSmartDashboard();
    m_backLeft.updateSmartDashboard();
    m_backRight.updateSmartDashboard();
    // SmartDashboard.putNumber("Robot Pitch", m_gyro.getPitch().getDegrees());
    // SmartDashboard.putNumber("CurrentPoseX", fieldPoseX);
    // SmartDashboard.putNumber("CurrentPoseY", fieldPoseY);
    // SmartDashboard.putData(currentFieldPos);
    SmartDashboard.putNumber("CurrentAngle", m_gyro.getYaw().getDegrees());
  }

  @Override
  public void setUpTrackables() {
    int loggingFrequency = 5;
    logger.addStringTrackable(
        () -> (fieldPoseX + "," + fieldPoseY + "," + m_gyro.getYaw().getDegrees()),
        "Swerve_Drive_Position",
        loggingFrequency,
        "Robot_X_Coordinate,Robot_Y_Coordinate,Robot_Angle_(degrees)");

    logger.addStringTrackable(
        () -> (visionEstimatedX + "," + visionEstimatedY),
        "Vision_Position_Reset_Coordinates",
        loggingFrequency,
        "Vision_Est_X,Vision_Est_Y");
    /*
    Keeping this here for now in case we need to sort logging files by data type

    logger.addStringTrackable(
        () ->
            (m_frontLeft.getDesiredSpeed()
                + ","
                + m_frontRight.getDesiredSpeed()
                + ","
                + m_backLeft.getDesiredSpeed()
                + ","
                + m_backRight.getDesiredSpeed()),
        "Swerve Module Desired Speeds",
        loggingFrequency,
        "Front Left Desired Speed, Front Right Desired Speed, Back Left Desired Speed, Back Right Desired Speed");
    logger.addStringTrackable(
        () ->
            (m_frontLeft.getDesiredAngle()
                + ","
                + m_frontRight.getDesiredAngle()
                + ","
                + m_backLeft.getDesiredAngle()
                + ","
                + m_backRight.getDesiredAngle()),
        "Swerve Module Desired Angles",
        loggingFrequency,
        "Front Left Desired Angle, Front Right Desired Angle, Back Left Desired Angle, Back Right Desired Angle");
    logger.addStringTrackable(
        () ->
            (m_frontLeft.getSpeed()
                + ","
                + m_frontRight.getSpeed()
                + ","
                + m_backLeft.getSpeed()
                + ","
                + m_backRight.getSpeed()),
        "Swerve Module Actual Speeds",
        loggingFrequency,
        "Front Left Actual Speed, Front Right Actual Speed, Back Left Actual Speed, Back Right Actual Speed");
    logger.addStringTrackable(
        () ->
            (m_frontLeft.getAbsoluteAngleDegrees()
                + ","
                + m_frontRight.getAbsoluteAngleDegrees()
                + ","
                + m_backLeft.getAbsoluteAngleDegrees()
                + ","
                + m_backRight.getAbsoluteAngleDegrees()),
        "Swerve Module Actual Angles",
        loggingFrequency,
        "Front Left Actual Angle, Front Right Actual Angle, Back Left Actual Angle, Back Right Actual Angle");
    */
  }

  public SwerveDriveKinematics getKinematics() {
    return m_kinematics;
  }

  public Pose2d getOdometry() {
    return m_odometry.getPoseMeters();
  }

  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
    };
  }

  /**
   * Used to directly set the state of (and move) the swerve modules
   *
   * @param desiredStates A list of desired states for each of the swerve modules, following the
   *     order front left, front right, back left, back right.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kMaxSpeed);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  public void consumeVisionEstimate(Pose2d visionMeasurement) {
    // Might use PoseEstimator later instead of just resetting odometry
    m_gyro.setYawAdjustment(visionMeasurement.getRotation());
    resetOdometry(visionMeasurement);
  }

  @Override
  public void periodic() {
    updateOdometry();
    currentPose = m_odometry.getPoseMeters();
    currentFieldPos.setRobotPose(currentPose);
    fieldPoseX = currentPose.getX();
    fieldPoseY = currentPose.getY();
  }

  /**
   * Sets your position on the field
   *
   * @param pose The position on the field you want the robot to think it's at
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getYaw(), getModulePositions(), pose);
  }
}
