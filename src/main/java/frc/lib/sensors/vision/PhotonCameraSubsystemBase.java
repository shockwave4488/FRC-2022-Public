package frc.lib.sensors.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.logging.Logger;
import frc.lib.sensors.vision.VisionTargets.PhotonTarget;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import java.util.stream.Collectors;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonCameraSubsystemBase<T extends PhotonTarget> extends VisionCamera<T> {
  protected final PhotonCamera camera;
  protected PhotonPipelineResult curResult;
  private final Function<PhotonTrackedTarget, T> targetMaker;

  PhotonCameraSubsystemBase(
      String name,
      CameraPositionConstants cameraConsts,
      Logger logger,
      Function<PhotonTrackedTarget, T> targetMaker) {
    super(name, cameraConsts, logger);
    camera = new PhotonCamera(name);
    this.targetMaker = targetMaker;
  }

  @Override
  public boolean hasTargets() {
    return getCurResult().hasTargets();
  }

  @Override
  /** Returns null if the camera doesn't see any targets */
  public Optional<T> getBestTarget() {
    return Optional.ofNullable(
        getCurResult().hasTargets() ? targetMaker.apply(getCurResult().getBestTarget()) : null);
  }

  @Override
  public List<T> getTargets() {
    return curResult.getTargets().stream()
        .map(target -> targetMaker.apply(target))
        .collect(Collectors.toUnmodifiableList());
  }

  @Override
  public int getRunningPipeline() {
    return camera.getPipelineIndex();
  }

  @Override
  public void setPipeline(int index) {
    camera.setPipelineIndex(index);
  }

  @Override
  public void setLed(VisionLEDMode mode) {
    camera.setLED(mode);
  }

  @Override
  public void takeSnapshot() {
    camera.takeOutputSnapshot();
  }

  private PhotonPipelineResult getCurResult() {
    if (curResult == null) {
      curResult = camera.getLatestResult();
    }
    return curResult;
  }

  @Override
  public void periodic() {
    curResult = camera.getLatestResult();
  }

  @Override
  public void updateSmartDashboard() {
    super.updateSmartDashboard();
    SmartDashboard.putBoolean("Camera has targets", getCurResult().hasTargets());
    SmartDashboard.putNumber("Latest result timestamp", getCurResult().getTimestampSeconds());
  }
}
