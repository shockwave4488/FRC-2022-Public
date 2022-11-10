package frc.robot.commands.c2022.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.flowcontrol.EdgeTrigger;
import frc.lib.sensors.Limelight;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.c2022.Indexer;
import frc.robot.subsystems.c2022.Shooter;
import java.util.function.Supplier;

public class SpinFlywheel extends CommandBase {
  private final Shooter shooter;
  private final Limelight limelight;
  private final Indexer indexer;
  private final Supplier<Pose2d> pose;

  private final boolean estimate;
  private final boolean end;
  private final boolean interpolateShooter;
  private double rpmOffset;
  private double hoodOffset;
  private double lastSpeed = Constants.ShooterConstants.FENDER_RPM;
  private static final double FALLBACK_SPEED = Constants.ShooterConstants.BACK_OF_TARMAC_RPM;
  private static final double FALLBACK_HOOD_INPUT =
      Constants.ShooterConstants.BACK_OF_TARMAC_HOOD_INPUT;
  private EdgeTrigger flywheelBB = new EdgeTrigger();

  // Should be moved to/taken from prefs.
  private static final double GROUND_TO_TAPE = 103;
  private static final double CAM_HEIGHT = 41;
  private static final double LIMELIGHT_ANGLE = 25;
  private static final double METERS_TO_INCHES = 39.37;

  public SpinFlywheel(
      Shooter shooter,
      Limelight limelight,
      Indexer indexer,
      Supplier<Pose2d> pose,
      boolean estimate,
      boolean interpolateShooter,
      boolean end) {
    this.shooter = shooter;
    this.limelight = limelight;
    this.indexer = indexer;
    this.pose = pose;
    this.estimate = estimate;
    this.end = end;
    this.interpolateShooter = interpolateShooter;
    addRequirements(shooter);
    addRequirements(limelight);
  }

  @Override
  public void initialize() {
    spinToSpeed();
    /*
    if (limelight != null) {
      limelight.setLed(LedControl.ForceOn);
    }
    */
    flywheelBB.update(indexer.getIndexerStates().getFlywheelBeamBreak());
  }

  @Override
  public void execute() {
    if (!end) {
      spinToSpeed();
    }
  }

  private void spinToSpeed() {
    double speed;
    rpmOffset = shooter.getRPMOffset();
    hoodOffset = shooter.getHoodOffset();

    speed = getSpeed();
    lastSpeed = speed;

    speed += rpmOffset;
    // SmartDashboard.putNumber("SpinFlywheel RPM (offset)", speed);

    if (interpolateShooter) {
      shooter.setRPM(speed);
    }
  }

  private double getSpeed() {
    double speed;
    double yOffset;

    if (limelight.hasTarget()) {
      yOffset = limelight.getY();
    } else {
      if (estimate) {
        yOffset = groundToYOffset();
      } else {
        if (end) {
          shooter.setHoodPosition(FALLBACK_HOOD_INPUT);
          return FALLBACK_SPEED - rpmOffset;
        } else {
          return lastSpeed;
        }
      }
    }

    speed = shooter.getRPMFromYOffset(yOffset);

    double hoodLevel = shooter.getHoodLevelFromYOffset(yOffset);
    hoodLevel += hoodOffset;
    shooter.setHoodPosition(hoodLevel);
    if (flywheelBB.getFallingUpdate(indexer.getIndexerStates().getFlywheelBeamBreak())) {
      shooter.logShooterValues(yOffset, speed + rpmOffset, hoodLevel);
    }

    return speed;
  }

  private double groundToYOffset() {
    // Checked using actual values, and off by a few degrees, likely due to measurement error

    double ground_dist =
        pose.get().getTranslation().getDistance(Constants.FieldConstants.HUB_CENTER)
            - FieldConstants.HUB_RADIUS_METERS;
    ground_dist *= METERS_TO_INCHES;
    double angleToLimelight =
        Math.atan((GROUND_TO_TAPE - CAM_HEIGHT) / ground_dist) * 180 / Math.PI;
    double yOffset = angleToLimelight - LIMELIGHT_ANGLE;
    return yOffset;
  }

  @Override
  public boolean isFinished() {
    return end ? shooter.isReady() : false;
  }

  @Override
  public void end(boolean interrupted) {
    /*
    if (limelight != null) {
      limelight.setLed(LedControl.ForceOff);
    }
    */
  }
}
