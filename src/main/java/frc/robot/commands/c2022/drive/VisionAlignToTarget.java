package frc.robot.commands.c2022.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.sensors.Limelight;
import frc.lib.sensors.NavX;
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
  private double currentHasTargetCycles = 0;
  private double currentAtSetpointCycles = 0;
  private double currentResetCycles = 0;
  private static final double DEFAULT_DONE_TOLERANCE = 2.5;
  private double currentTolerance = DEFAULT_DONE_TOLERANCE;
  /* limelightXAngle & limelightYAngle are set to illogical values to guarantee that
  hasSignificantChange returns true and that these values are updated upon the first cycle that
  the limelight sees the target */
  private double limelightXAngle = 999;
  private double limelightYAngle = 999;
  private static final double MIN_AT_SETPOINT_CYCLES = 10;
  private static final double MIN_RESET_POS_CYCLES = 4;
  /* prevents setpoint from being updated if the change in limelight.getX() is less than this value. */
  private static final double ANTI_OSCILLATION_THRESHOLD = 1;
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
    currentTolerance = DEFAULT_DONE_TOLERANCE;
    currentHasTargetCycles = 0;
    currentAtSetpointCycles = 0;
    currentResetCycles = 0;
  }

  @Override
  public void execute() {
    double rotPower = 0;

    /*
    pidController.setP(SmartDashboard.getNumber("LLPID_P", DEFAULT_PID_P));
    pidController.setI(SmartDashboard.getNumber("LLPID_I", DEFAULT_PID_I));
    pidController.setD(SmartDashboard.getNumber("LLPID_D", DEFAULT_PID_D));
    */

    if (limelight.hasTarget()) {
      if (hasSignificantChange(limelight.getX(), limelightXAngle)) {
        limelightXAngle = limelight.getX();
        // Returns a negative value when the target is on the left side of the screen
      }
      if (hasSignificantChange(limelight.getY(), limelightYAngle)) {
        limelightYAngle = limelight.getY();
        double translationalDist =
            limelight.getEstimatedDistance() + FieldConstants.HUB_RADIUS_METERS;
        currentTolerance =
            Math.abs(Math.atan(FieldConstants.HUB_SAFE_SHOT_RADIUS_METERS / translationalDist));
        currentTolerance *= 180 / Math.PI; // convert to degrees
      }
      double currentAngle = gyro.getYaw().getDegrees();
      // SmartDashboard.putNumber("LL X Difference", limelightXAngle);
      double targetAngle = currentAngle - limelightXAngle;
      if (targetAngle > 180) {
        targetAngle -= 360;
      } else if (targetAngle < -180) {
        targetAngle += 360;
      }
      // SmartDashboard.putNumber("LL Target Angle", targetAngle);
      rotPower = pidController.calculate(currentAngle, targetAngle);
      rotPower *= rotationMultiplier;
      rotPower = Math.min(Math.max(-rotationMultiplier, rotPower), rotationMultiplier);
    }

    // tolerance calculated based on distance so we can have a prediction if we'll make our shot
    // based on our pidController
    pidController.setTolerance(currentTolerance);

    if (limelight.hasTarget()) {
      currentHasTargetCycles++;
    }

    atSetpoint = pidController.atSetpoint();
    if (atSetpoint) {
      currentAtSetpointCycles++;
    } else {
      currentAtSetpointCycles = 0;
    }

    double[] driveSpeedValues = driveValues.get();
    double driveXSpeed = driveSpeedValues[0] * speedMultiplier;
    double driveYSpeed = driveSpeedValues[1] * speedMultiplier;

    currentResetCycles++;
    // Condition under which it should be safe to reset the position of the swerve drive based on
    // vision and gyro.
    if ((readyToShoot())
        && limelight.hasTarget()
        && driveXSpeed == 0
        && driveYSpeed == 0
        && (currentResetCycles > MIN_RESET_POS_CYCLES)) {
      swerve.updateEstimatedPosition(limelight.getEstimatedDistance());
      currentResetCycles = 0;
      /* set to 0 to make it so only 4 cycles are needed before the pose estimation runs
      again. This helps achieve a balance (although arbitrary) between keeping the
      estimated pose up to date and CPU consumption. */
    }

    swerve.drive(driveXSpeed, driveYSpeed, rotPower, true);
  }

  private boolean hasSignificantChange(double currentLimelightValue, double storedLimelightValue) {
    return Math.abs((currentLimelightValue - storedLimelightValue)) > ANTI_OSCILLATION_THRESHOLD;
  }

  private boolean readyToShoot() {
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
