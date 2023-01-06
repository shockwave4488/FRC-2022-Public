package frc.robot.commands.eruption.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.sensors.NavX;
import frc.lib.sensors.vision.Limelight;
import frc.lib.util.app.PoseEstimationUtil;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import java.util.function.Supplier;

/**
 * This command will take control of the swerve drive's rotation to make the swerve drive face a
 * vision target. It will only end itself once it is facing the target. So, this command should be
 * ran in parallel with another command that finishes if the limelight loses the vision target.
 */
public class VisionAlignToTarget extends CommandBase {
  private final SwerveDrive swerve;
  private final Limelight limelight;
  private final NavX gyro;
  private final double speedMultiplier;
  private final double rotationMultiplier;
  private final Supplier<double[]> driveValues;
  private final int minDoneCycles;
  private final boolean finish;
  private boolean atSetpoint = false;
  private double currentHasTargetCycles;
  private double currentAtSetpointCycles;
  private boolean hasResetPose;
  private static final double DEFAULT_DONE_TOLERANCE = 2.5;
  private double currentTolerance = DEFAULT_DONE_TOLERANCE;
  private Rotation2d limelightXAngle;
  private Rotation2d limelightYAngle;
  private double translationalDist;
  private static final double MIN_AT_SETPOINT_CYCLES = 10;
  /* prevents setpoint from being updated if the change in limelight.getX() is less than this value. */
  private static final double ANTI_OSCILLATION_THRESHOLD = 1; // degrees
  private static final double DEFAULT_PID_P = DriveTrainConstants.SWERVE_DRIVE_ROTATION_P;
  private static final double DEFAULT_PID_I = DriveTrainConstants.SWERVE_DRIVE_ROTATION_I;
  private static final double DEFAULT_PID_D = DriveTrainConstants.SWERVE_DRIVE_ROTATION_D;
  private PIDController pidController =
      new PIDController(DEFAULT_PID_P, DEFAULT_PID_I, DEFAULT_PID_D);

  /**
   * A command that rotates the robot to face a vision target. Thank you to 2910's
   * VisionRotateToTargetCommand class (found here
   * https://github.com/FRCTeam2910/2021CompetitionRobot/blob/master/src/main/java/org/frcteam2910/c2020/commands/VisionRotateToTargetCommand.java)
   * for providing an initial inspiration for this command.
   *
   * @param swerve
   * @param limelight
   * @param gyro
   */
  public VisionAlignToTarget(
      SwerveDrive swerve,
      Limelight limelight,
      NavX gyro,
      double speedMultiplier,
      double rotationMultiplier,
      Supplier<double[]> driveValues,
      int minDoneCycles,
      boolean finish) {
    this.swerve = swerve;
    this.limelight = limelight;
    this.gyro = gyro;
    this.speedMultiplier = speedMultiplier;
    this.rotationMultiplier = rotationMultiplier;
    this.driveValues = driveValues;
    this.minDoneCycles = minDoneCycles;
    this.finish = finish;

    pidController.enableContinuousInput(-180, 180);
    pidController.setTolerance(DEFAULT_DONE_TOLERANCE);

    /*
    SmartDashboard.putNumber("LLPID_P", DEFAULT_PID_P);
    SmartDashboard.putNumber("LLPID_I", DEFAULT_PID_I);
    SmartDashboard.putNumber("LLPID_D", DEFAULT_PID_D);
    */

    // Shouldn't require the limelight here because no changes are applied to it.
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    pidController.reset();
    /* limelightXAngle & limelightYAngle are set to illogical values to guarantee that
    hasSignificantChange returns true and that these values are updated upon the first cycle that
    the limelight sees the target */
    limelightXAngle = new Rotation2d(Math.PI);
    limelightYAngle = new Rotation2d(Math.PI);
    currentTolerance = DEFAULT_DONE_TOLERANCE;
    currentHasTargetCycles = 0;
    currentAtSetpointCycles = 0;
  }

  @Override
  public void execute() {
    double rotPower = 0;
    boolean hasTarget = limelight.hasTargets();

    if (hasTarget) {
      Rotation2d actualLimelightX = limelight.getBestTarget().get().getX();
      Rotation2d actualLimelightY = limelight.getBestTarget().get().getY();
      if (hasSignificantChange(actualLimelightX, limelightXAngle)) {
        limelightXAngle = actualLimelightX;
        // Returns a positive value when the target is on the left side of the screen
      }
      if (hasSignificantChange(actualLimelightY, limelightYAngle)) {
        limelightYAngle = actualLimelightY;
        translationalDist =
            limelight.getEstimatedDistance(
                    limelight.getBestTarget().get(), FieldConstants.TARGET_HEIGHT_METERS)
                + FieldConstants.HUB_RADIUS_METERS;
        currentTolerance =
            Math.toDegrees(
                Math.abs(
                    Math.atan(FieldConstants.HUB_SAFE_SHOT_RADIUS_METERS / translationalDist)));
        hasResetPose = false;
      }
      Rotation2d currentAngle = gyro.getYaw();
      // SmartDashboard.putNumber("LL X Difference", limelightXAngle);
      double targetAngle = currentAngle.plus(limelightXAngle).getDegrees();
      if (targetAngle > 180) {
        targetAngle -= 360;
      } else if (targetAngle < -180) {
        targetAngle += 360;
      }
      // SmartDashboard.putNumber("LL Target Angle", targetAngle);
      rotPower = pidController.calculate(currentAngle.getDegrees(), targetAngle);
      rotPower *= rotationMultiplier;
      rotPower = Math.min(Math.max(-rotationMultiplier, rotPower), rotationMultiplier);

      currentHasTargetCycles++;
    }

    // tolerance calculated based on distance so we can have a prediction if we'll make our shot
    // based on our pidController
    pidController.setTolerance(currentTolerance);

    atSetpoint = pidController.atSetpoint();
    if (atSetpoint) {
      currentAtSetpointCycles++;
    } else {
      currentAtSetpointCycles = 0;
    }

    double[] driveSpeedValues = driveValues.get();
    double driveXSpeed = driveSpeedValues[0] * speedMultiplier;
    double driveYSpeed = driveSpeedValues[1] * speedMultiplier;

    // Condition under which it should be safe to reset the position of the swerve drive based on
    // vision and gyro.
    if (!hasResetPose && hasTarget && readyToShoot() && driveXSpeed == 0 && driveYSpeed == 0) {
      PoseEstimationUtil.consumePoseFrom2d(
          swerve,
          FieldConstants.HUB_CENTER,
          translationalDist,
          gyro.getYaw().plus(limelight.getBestTarget().get().getX()));
      hasResetPose = true;
    }

    swerve.drive(driveXSpeed, driveYSpeed, rotPower, true);
  }

  private boolean hasSignificantChange(
      Rotation2d currentLimelightValue, Rotation2d storedLimelightValue) {
    return Math.abs((currentLimelightValue.getDegrees() - storedLimelightValue.getDegrees()))
        > ANTI_OSCILLATION_THRESHOLD;
  }

  public boolean readyToShoot() {
    return ((currentAtSetpointCycles > MIN_AT_SETPOINT_CYCLES)
        && (currentHasTargetCycles > minDoneCycles));
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(0, 0, 0, true);
  }

  @Override
  public boolean isFinished() {
    return finish && readyToShoot();
  }
}
