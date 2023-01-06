package frc.lib.sensors.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.logging.Logger;
import frc.lib.sensors.vision.VisionTargets.VisionTarget;
import frc.lib.util.app.PoseEstimationUtil;
import frc.lib.wpiextensions.ShockwaveSubsystemBase;
import java.util.List;
import java.util.Optional;
import org.photonvision.common.hardware.VisionLEDMode;

// TODO: Modify Limelight parameters throughout the code to use appropriate base class or interface
public abstract class VisionCamera<T extends VisionTarget> extends ShockwaveSubsystemBase {
  public static class CameraPositionConstants {
    /** Height of the camera off the ground in meters */
    public final double camHeight;
    /** Pitch of the camera, where 0 is level with the ground, and pointed upwards is negative */
    public final Rotation2d camToNormalAngle;
    /**
     * Transform representing position (in meters) and orientation of the camera (relative to the
     * robot)
     */
    public final Transform3d robotCenterToCamera;

    /**
     * @param robotCenterToCamera Transform representing position (in meters) and orientation of the
     *     camera (relative to the robot)
     */
    public CameraPositionConstants(Transform3d robotCenterToCamera) {
      camHeight = robotCenterToCamera.getZ();
      camToNormalAngle = new Rotation2d(robotCenterToCamera.getRotation().getY());
      this.robotCenterToCamera = robotCenterToCamera;
    }
  }

  protected final String name;
  protected final Logger logger;
  protected final CameraPositionConstants cameraConsts;

  protected SendableChooser<VisionLEDMode> forceLedSelector = new SendableChooser<>();
  protected VisionLEDMode currentLedSelection = forceLedSelector.getSelected();

  public VisionCamera(String name, CameraPositionConstants cameraConsts, Logger logger) {
    this.name = name;
    this.logger = logger;
    this.cameraConsts = cameraConsts;

    forceLedSelector.setDefaultOption(name + "-NormalLedControl", VisionLEDMode.kDefault);
    forceLedSelector.addOption(name + "-ForceLedOn", VisionLEDMode.kOn);
    forceLedSelector.addOption(name + "-ForceLedOff", VisionLEDMode.kOff);
    forceLedSelector.addOption(name + "-LedBlink", VisionLEDMode.kBlink);
    SmartDashboard.putData(forceLedSelector);
  }

  public CameraPositionConstants getCameraPositionConsts() {
    return cameraConsts;
  }

  /**
   * Estimates the translational distance (not including height) from the limelight to the target
   * (tape, tag, etc.). Works better the greater the height differential is between the target and
   * the robot.
   *
   * @param target Vision target to estimate distance to
   * @param targetHeight of the target in meters
   * @return Translational distance to target in meters
   */
  public double getEstimatedDistance(VisionTarget target, double targetHeight) {
    return PoseEstimationUtil.calculateDistanceToTarget(
        cameraConsts.camHeight,
        targetHeight,
        cameraConsts.camToNormalAngle,
        target.getY(),
        target.getX());
  }

  /**
   * @param target Vision target to turn to
   * @param currentAngle The current angle of the robot
   * @param defaultAngle The rotation to turn to if {@code target} is null
   * @return If the camera has a target, the angle needed to face the center of that target,
   *     otherwise, this returns a default angle.
   */
  public Rotation2d getDesiredAngle(
      VisionTarget target, Rotation2d currentAngle, Rotation2d defaultAngle) {
    Rotation2d targetAngle = defaultAngle;
    if (target != null) {
      Rotation2d yawOffset = target.getX();
      targetAngle = currentAngle.minus(yawOffset);
    }
    return targetAngle;
  }

  public abstract boolean hasTargets();

  public abstract void takeSnapshot();

  public abstract Optional<T> getBestTarget();

  public abstract List<T> getTargets();

  public abstract int getRunningPipeline();

  public abstract void setPipeline(int index);

  public abstract void setLed(VisionLEDMode mode);

  @Override
  public void onStart() {}

  @Override
  public void onStop() {}

  @Override
  public void zeroSensors() {}

  @Override
  public void periodic() {}

  @Override
  public void updateSmartDashboard() {
    VisionLEDMode ledSelection = forceLedSelector.getSelected();
    if (ledSelection != currentLedSelection) {
      currentLedSelection = ledSelection;
      setLed(ledSelection);
    }
  }

  @Override
  public void setUpTrackables() {
    logger.addTrackable(() -> hasTargets() ? 1 : 0, name + "_HasTarget", 10);
  }
}
