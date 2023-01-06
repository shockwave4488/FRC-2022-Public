package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drive.SwerveParameters.ModulePosition;
import frc.lib.logging.Logger;
import frc.lib.sensors.NavX;
import frc.lib.util.app.Util;
import frc.lib.wpiextensions.ShockwaveSubsystemBase;
import frc.robot.Constants.FieldConstants;
import java.util.HashMap;
import java.util.Map;

/** Represents a swerve drive style drivetrain. */
public class SwerveDrive extends ShockwaveSubsystemBase {
  public static final double kMaxSpeed = 3.5; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // rotations per second

  private static final double kMinimumInputValue = 0.01;

  private final Translation2d[] moduleLocations;
  private final ISwerveModule[] SModules;

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
   * @param parameters Parameters that are passed to SwerveModules for their setup
   */
  public SwerveDrive(NavX gyro, ISwerveModule[] modules, Logger logger) {
    m_gyro = gyro;
    this.logger = logger;
    SModules = modules;

    moduleLocations = new Translation2d[SModules.length];
    for (int i = 0; i < SModules.length; i++) {
      moduleLocations[i] = SModules[i].getLocation();
    }
    m_kinematics = new SwerveDriveKinematics(moduleLocations);
    m_odometry =
        new SwerveDriveOdometry(m_kinematics, new Rotation2d(m_gyro.getYaw().getRadians()));
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
    if (Util.epsilonEquals(xSpeed, 0, kMinimumInputValue)
        && Util.epsilonEquals(ySpeed, 0, kMinimumInputValue)
        && Util.epsilonEquals(rot, 0, kMinimumInputValue)) {
      for (ISwerveModule module : SModules) {
        module.halt();
      }
    }

    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, new Rotation2d(m_gyro.getYaw().getRadians()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    setModuleStates(swerveModuleStates);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[SModules.length];
    for (int i = 0; i < SModules.length; i++) {
      moduleStates[i] = SModules[i].getState();
    }
    m_odometry.update(new Rotation2d(m_gyro.getYaw().getRadians()), moduleStates);
  }

  @Override
  public void onStart() {
    for (ISwerveModule module : SModules) {
      module.onStart();
    }
  }

  @Override
  public void onStop() {}

  @Override
  public void zeroSensors() {
    m_gyro.reset();
  }

  @Override
  public void updateSmartDashboard() {
    for (ISwerveModule module : SModules) {
      module.updateSmartDashboard();
    }
    // SmartDashboard.putNumber("Robot Pitch", m_gyro.getPitch().getDegrees());
    SmartDashboard.putNumber("CurrentPoseX", fieldPoseX);
    SmartDashboard.putNumber("CurrentPoseY", fieldPoseY);
    SmartDashboard.putData(currentFieldPos);
    SmartDashboard.putNumber("CurrentAngle", m_gyro.getYaw().getDegrees());
  }

  /*
  private String trackableStr(String property) {
    String tStr = "";

    for (int i = 0; i < SModules.length; i++) {
      if (property.equals("Speed")) {
        tStr += SModules[i].getDesiredSpeed();
      } else if (property.equals("DAngle")) {
        tStr += SModules[i].getDesiredAngle();
      } else if (property.equals("ASpeed")) {
        tStr += SModules[i].getSpeed();
      } else if (property.equals("AAngle")) {
        tStr += SModules[i].getAbsoluteAngleDegrees();
      }

      if (i < SModules.length - 1) {
        tStr += ",";
      }
    }
    return tStr;
  }
  */

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
        () -> (trackableStr("DSpeed")),
        "Swerve Module Desired Speeds",
        loggingFrequency,
        "Front Left Desired Speed, Front Right Desired Speed, Back Left Desired Speed, Back Right Desired Speed");
    logger.addStringTrackable(
        () -> (trackableStr("DAngle")),
        "Swerve Module Desired Angles",
        loggingFrequency,
        "Front Left Desired Angle, Front Right Desired Angle, Back Left Desired Angle, Back Right Desired Angle");
    logger.addStringTrackable(
        () -> (trackableStr("ASpeed")),
        "Swerve Module Actual Speeds",
        loggingFrequency,
        "Front Left Actual Speed, Front Right Actual Speed, Back Left Actual Speed, Back Right Actual Speed");
    logger.addStringTrackable(
        () -> (trackableStr("AAngle")),
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

  /**
   * Used to directly set the state of (and move) the swerve modules
   *
   * @param desiredStates A list of desired states for each of the swerve modules, following the
   *     order in which they were created in the robot container (passed into Kinematics).
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kMaxSpeed);
    for (int i = 0; i < SModules.length; i++) {
      SModules[i].setDesiredState(desiredStates[i]);
    }
  }

  public void setModuleStates(Map<ModulePosition, SwerveModuleState> desiredStates) {
    Map<String, SwerveModuleState> desiredStatesStr = new HashMap<String, SwerveModuleState>();
    for (Map.Entry<ModulePosition, SwerveModuleState> state : desiredStates.entrySet()) {
      desiredStatesStr.put(state.getKey().toString(), state.getValue());
    }

    SwerveModuleState[] desiredStatesArray = new SwerveModuleState[SModules.length];
    for (int i = 0; i < SModules.length; i++) {
      SwerveModuleState desiredState = desiredStatesStr.get(SModules[i].getRobotPosition());
      desiredStatesArray[i] = desiredState;
    }
    setModuleStates(desiredStatesArray);
  }

  /**
   * This method will calculate an estimated robot position based on a given translational distance
   * and internal gyro knowledge. Only call this method if you know your translational distance is
   * correct.
   *
   * @param translationalDistance Translational distance from the target in meters
   */
  public void updateEstimatedPosition(double translationalDistance) {
    translationalDistance += FieldConstants.HUB_RADIUS_METERS;
    SmartDashboard.putNumber("PR Translational Distance (HUB Center)", translationalDistance);

    // Yes the gyro angle may be better to use but theoretically it and the pose angle should be
    // identical, and it wouldn't be ideal to make a new rotation 2d every cycle to make the new
    // pose. And from testing, the gyro and odometry angles are identical, as they should be.
    Rotation2d currentRotation = currentPose.getRotation();
    double currentAngle = currentRotation.getRadians();

    // The robot's quadrant on the field does not matter when doing the following calculations.
    double relativeX = -Math.cos(currentAngle) * translationalDistance;
    double relativeY = -Math.sin(currentAngle) * translationalDistance;
    visionEstimatedX = FieldConstants.HUB_CENTER.getX() + relativeX;
    visionEstimatedY = FieldConstants.HUB_CENTER.getY() + relativeY;
    SmartDashboard.putNumber("PR Estimated X", visionEstimatedX);
    SmartDashboard.putNumber("PR Estimated Y", visionEstimatedY);

    Pose2d estimatedPose = new Pose2d(visionEstimatedX, visionEstimatedY, currentRotation);
    resetOdometry(estimatedPose);
    logger.writeToLogFormatted(
        this,
        "Updated pose with vision, now X = " + visionEstimatedX + ", Y = " + visionEstimatedY);
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
    m_odometry.resetPosition(pose, new Rotation2d(m_gyro.getYaw().getRadians()));
  }
}
