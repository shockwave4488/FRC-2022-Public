package frc.lib.sensors.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.logging.Logger;
import frc.lib.sensors.vision.VisionTargets.AprilTagPhotonTarget;
import frc.lib.sensors.vision.VisionTargets.PhotonTarget;

public class PhotonCameras {
  private PhotonCameras() {}

  public static class AprilTagPhotonCamera extends PhotonCameraSubsystemBase<AprilTagPhotonTarget> {
    public AprilTagPhotonCamera(String name, CameraPositionConstants cameraConsts, Logger logger) {
      super(name, cameraConsts, logger, AprilTagPhotonTarget::new);
    }

    @Override
    public void updateSmartDashboard() {
      super.updateSmartDashboard();
      if (getBestTarget().isPresent()) {
        SmartDashboard.putNumber("Best target ID", getBestTarget().get().getId());
        SmartDashboard.putString(
            "Best target transform", getBestTarget().get().getCameraToTarget().toString());
      } else {
        SmartDashboard.putNumber("Best target ID", -1);
        SmartDashboard.putString("Best target transform", "No target");
      }
      SmartDashboard.putNumberArray(
          "Target IDs", getTargets().stream().mapToDouble(target -> target.getId()).toArray());
    }
  }

  public static class PhotonCameraSubsystem extends PhotonCameraSubsystemBase<PhotonTarget> {
    public PhotonCameraSubsystem(String name, CameraPositionConstants cameraConsts, Logger logger) {
      super(name, cameraConsts, logger, PhotonTarget::new);
    }
  }
}
