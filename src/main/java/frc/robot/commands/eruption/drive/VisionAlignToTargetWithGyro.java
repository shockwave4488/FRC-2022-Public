package frc.robot.commands.eruption.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.sensors.NavX;
import frc.lib.sensors.vision.Limelight;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveDrive;
import java.util.function.Supplier;

/**
 * This command will take control of the swerve drive's rotation to make the swerve drive face a
 * vision target. It will only end itself once it is facing the target. So, this command should be
 * ran in parallel with another command that finishes if the limelight loses the vision target.
 */
public class VisionAlignToTargetWithGyro extends CommandBase {
  private final SwerveDrive swerve;
  private final Limelight limelight;
  private final NavX gyro;
  private final double speedMultiplier;
  private final double rotationMultiplier;
  private final Supplier<double[]> driveValues;
  private int minDoneCycles = 10;
  private static final int MIN_GYRO_DONE_CYCLES = 20;
  private static final int MIN_GYRO_DONE_CYCLES_UNTIL_SWITCH =
      5; // When gyro mode is steady enough to transition to vision mode
  private double currentCyclesLL = 0;
  private double currentCyclesGyro = 0;
  private boolean continuous; // True means hub-centric

  // private PIDController pidController = new PIDController(0.2, 2.0, 0.025); // constants from
  // 2910
  private PIDController pidControllerLL = new PIDController(0.035, 0, 0.002);
  private PIDController pidControllerGyro = new PIDController(0.5, 0, 0); // needs tuning

  /**
   * A command that rotates the robot to face a vision target. Thank you to 2910's
   * VisionRotateToTargetCommand class (found here
   * https://github.com/FRCTeam2910/2021CompetitionRobot/blob/master/src/main/java/org/frcteam2910/c2020/commands/VisionRotateToTargetCommand.java)
   * for heavily inspiring this command.
   *
   * <p>Also note that this class is still in development and hasn't been tested.
   *
   * @param swerve
   * @param limelight
   * @param gyro
   */
  public VisionAlignToTargetWithGyro(
      SwerveDrive swerve,
      Limelight limelight,
      NavX gyro,
      double speedMultiplier,
      double rotationMultiplier,
      Supplier<double[]> driveValues,
      boolean continuous) {
    this.swerve = swerve;
    this.limelight = limelight;
    this.gyro = gyro;
    this.speedMultiplier = speedMultiplier;
    this.rotationMultiplier = rotationMultiplier;
    this.driveValues = driveValues;
    this.continuous = continuous;

    // Shouldn't require the limelight here because no changes are applied to it.
    addRequirements(swerve);
  }

  public VisionAlignToTargetWithGyro(
      SwerveDrive swerve,
      Limelight limelight,
      NavX gyro,
      double speedMultiplier,
      double rotationMultiplier,
      Supplier<double[]> driveValues,
      boolean continuous,
      int minDoneCycles) {
    this.swerve = swerve;
    this.limelight = limelight;
    this.gyro = gyro;
    this.speedMultiplier = speedMultiplier;
    this.rotationMultiplier = rotationMultiplier;
    this.driveValues = driveValues;
    this.continuous = continuous;
    this.minDoneCycles = minDoneCycles;

    // Shouldn't require the limelight here because no changes are applied to it.
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    pidControllerLL.enableContinuousInput(-180, 180);
    pidControllerLL.setTolerance(2);
    pidControllerGyro.enableContinuousInput(-180, 180);
    pidControllerGyro.setTolerance(3);
    pidControllerLL.reset();
    pidControllerGyro.reset();
  }

  @Override
  public void execute() {
    double rotPower = 0;
    SmartDashboard.putBoolean("LL Has Target", limelight.hasTargets());

    double gyroDistToHub =
        swerve.getOdometry().getTranslation().getDistance(Constants.FieldConstants.HUB_CENTER);
    double visbleHubRange = 15 * 4 / gyroDistToHub; // Range 15 in each direction at 4 meters away
    double gyroDesiredAngle = calcGyroDesiredAngle();
    boolean inVisbleHubRange =
        Math.abs(gyroDesiredAngle - gyro.getYaw().getDegrees()) <= visbleHubRange;

    if ((!continuous && (!inVisbleHubRange | currentCyclesGyro < MIN_GYRO_DONE_CYCLES_UNTIL_SWITCH))
        | (continuous && !inVisbleHubRange)) {
      // This is necessary because we can't be sure the target is the hub unless the robot is
      // pointing the right general direction.
      currentCyclesLL = 0;
      rotPower = gyroCalcPower(gyroDesiredAngle);
    } else if (!limelight.hasTargets()) {
      // Limelight relatively points to the hub but is probably too close to see it.
      // Likely finishes with gyro done cycles.
      currentCyclesLL = 0;
      rotPower = gyroCalcPower(gyroDesiredAngle);
    } else {
      // Robot is stable and seeing the target
      // Continuous mode (presumably driving around) skips to limelight mode regardless of whether
      // it is stably pointing at the hub.
      currentCyclesGyro = 0;
      rotPower = limelightCalcPower();
    }

    double[] driveSpeedValues = driveValues.get();
    double driveXSpeed = driveSpeedValues[0] * speedMultiplier;
    double driveYSpeed = driveSpeedValues[1] * speedMultiplier;
    swerve.drive(driveXSpeed, driveYSpeed, rotPower, true);
  }

  private double limelightCalcPower() {
    double currentAngle = gyro.getYaw().getDegrees();
    double limelightAngle = limelight.getBestTarget().get().getX().getDegrees();
    // Returns a positive value when the target is on the left side of the screen
    SmartDashboard.putNumber("LL X Difference", limelightAngle);
    SmartDashboard.putNumber("LL Robot Current Angle", currentAngle);
    double targetAngle = currentAngle + limelightAngle;
    if (targetAngle > 180) {
      targetAngle -= 360;
    } else if (targetAngle < -180) {
      targetAngle += 360;
    }
    SmartDashboard.putNumber("LL Target Angle", targetAngle);
    double rotPower = pidControllerLL.calculate(currentAngle, targetAngle);
    SmartDashboard.putNumber("LL Raw PID Output", rotPower);
    rotPower *= rotationMultiplier;
    rotPower = Math.min(Math.max(-rotationMultiplier, rotPower), rotationMultiplier);
    SmartDashboard.putNumber("LL Rot Value", rotPower);

    boolean atSetpoint = pidControllerLL.atSetpoint();
    rotPower = atSetpoint ? 0 : rotPower;
    if (!continuous) {
      if (atSetpoint) {
        currentCyclesLL++;
      } else {
        currentCyclesLL = 0;
      }
    }
    SmartDashboard.putBoolean("LL PID AtSetpoint", atSetpoint);

    return rotPower;
  }

  private double gyroCalcPower(double desiredAngle) {
    double currentAngle = gyro.getYaw().getDegrees();
    double rotPower = pidControllerGyro.calculate(currentAngle, desiredAngle);
    rotPower *= rotationMultiplier;
    rotPower = Math.min(Math.max(-rotationMultiplier, rotPower), rotationMultiplier);

    boolean atSetpoint = pidControllerGyro.atSetpoint();
    rotPower = atSetpoint ? 0 : rotPower;
    if (!continuous) {
      if (atSetpoint) {
        currentCyclesGyro++;
      } else {
        currentCyclesGyro = 0;
      }
    }
    SmartDashboard.putBoolean("Gyro PID AtSetpoint", atSetpoint);

    return rotPower;
  }

  private double calcGyroDesiredAngle() {
    Pose2d currentPose = swerve.getOdometry();
    double currentX = currentPose.getX();
    double currentY = currentPose.getY();
    double relativeX = Constants.FieldConstants.HUB_CENTER.getX() - currentX;
    double relativeY = Constants.FieldConstants.HUB_CENTER.getY() - currentY;

    if (relativeX == 0) {
      relativeX += 0.0001;
    }

    double desiredAngle = Math.atan2(relativeY, relativeX);

    double desiredAngleDegrees = desiredAngle * 180 / Math.PI;
    SmartDashboard.putNumber("Gyro Target Angle", desiredAngleDegrees);
    return desiredAngleDegrees;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(0, 0, 0, true);
  }

  @Override
  public boolean isFinished() {
    // Assuming limelight will take over before gyro cycles finish
    // Continuous does not count cycles
    return (currentCyclesLL >= minDoneCycles | currentCyclesGyro >= MIN_GYRO_DONE_CYCLES);
  }
}
