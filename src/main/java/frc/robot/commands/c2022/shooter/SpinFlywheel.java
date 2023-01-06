package frc.robot.commands.c2022.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.flowcontrol.EdgeTrigger;
import frc.lib.sensors.vision.Limelight;
import frc.lib.sensors.vision.VisionCamera.CameraPositionConstants;
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
  private final CameraPositionConstants cameraConsts;

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
    cameraConsts = limelight.getCameraPositionConsts();
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
      limelight.setLed(VisionLEDMode.kOn);
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

    if (limelight.hasTargets()) {
      yOffset = limelight.getBestTarget().get().getY().getDegrees();
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
    double ground_dist =
        pose.get().getTranslation().getDistance(Constants.FieldConstants.HUB_CENTER)
            - FieldConstants.HUB_RADIUS_METERS;
    double angleToLimelight =
        Math.atan((FieldConstants.TARGET_HEIGHT_METERS - cameraConsts.camHeight) / ground_dist);
    // Y-offset after being passed to a Rotation2d decreases as the target gets higher in the image,
    // but that's not what our interpolation tables use. To retain the old output, the camera pitch
    // (which is now negative when pointed up) must be added instead of subtracted.
    double yOffset = angleToLimelight + cameraConsts.camToNormalAngle.getDegrees();
    return Math.toDegrees(yOffset);
  }

  @Override
  public boolean isFinished() {
    return end ? shooter.isReady() : false;
  }

  @Override
  public void end(boolean interrupted) {
    /*
    if (limelight != null) {
      limelight.setLed(VisionLEDMode.kOff);
    }
    */
  }
}
