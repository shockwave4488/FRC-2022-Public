package frc.lib.sensors.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.util.app.PoseEstimationUtil;
import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class VisionTargets {
  private VisionTargets() {}

  public interface VisionTarget {
    /**
     * @return The horizontal angle difference between the center of the limelight's view and where
     *     its detected target is on its view. If the target is to the left of the center, the angle
     *     is positive.
     */
    Rotation2d getX();

    /**
     * @return The vertical angle difference between the center of the limelight's view and where
     *     its detected target is on its view. If the target is above the center, the angle is
     *     negative.
     */
    Rotation2d getY();

    /**
     * @return The percentage (0-100) of the image the target takes up.
     */
    double getArea();

    /**
     * @return The rotation of the bounding box of the target, in the range 0 to -90 degrees
     */
    Rotation2d getSkew();
  }

  public interface AprilTagTarget extends VisionTarget {
    int getId();

    Transform3d getCameraToTarget();

    double getPoseAmbiguity();

    default AprilTag toAprilTag(Pose2d robotPose, Transform3d robotCenterToCamera) {
      return new AprilTag(
          getId(),
          PoseEstimationUtil.getTargetPoseFromTransform(
              robotPose, getCameraToTarget(), robotCenterToCamera));
    }

    static AprilTag[] getVisibleAprilTags(
        VisionCamera<? extends AprilTagTarget> camera, Pose2d robotPose) {
      return camera.getTargets().stream()
          .map(
              tagTarget -> tagTarget.toAprilTag(robotPose, camera.cameraConsts.robotCenterToCamera))
          .toArray(AprilTag[]::new);
    }
  }

  public static class LimelightTarget implements VisionTarget {
    private final Rotation2d xOffset, yOffset, skew;
    private final double area;

    @Override
    public Rotation2d getX() {
      return xOffset;
    }

    @Override
    public Rotation2d getY() {
      return yOffset;
    }

    @Override
    public double getArea() {
      return area;
    }

    @Override
    public Rotation2d getSkew() {
      return skew;
    }

    LimelightTarget(double xOffset, double yOffset, double area, double skew) {
      this.xOffset = Rotation2d.fromDegrees(-xOffset);
      this.yOffset = Rotation2d.fromDegrees(-yOffset);
      this.skew = Rotation2d.fromDegrees(-skew); // guess, don't know if skew is CW or CCW
      this.area = area;
    }
  }

  public static class PhotonTarget implements VisionTarget {
    protected final PhotonTrackedTarget target;
    private final Rotation2d xOffset, yOffset, skew;

    @Override
    public Rotation2d getX() {
      return xOffset;
    }

    @Override
    public Rotation2d getY() {
      return yOffset;
    }

    @Override
    public double getArea() {
      return target.getArea();
    }

    @Override
    public Rotation2d getSkew() {
      return skew;
    }

    public List<TargetCorner> getCorners() {
      return target.getCorners();
    }

    PhotonTarget(PhotonTrackedTarget target) {
      this.target = target;
      xOffset = Rotation2d.fromDegrees(-target.getYaw());
      yOffset = Rotation2d.fromDegrees(-target.getPitch());
      skew = Rotation2d.fromDegrees(-target.getSkew());
    }
  }

  public static class AprilTagPhotonTarget extends PhotonTarget implements AprilTagTarget {
    @Override
    public int getId() {
      return target.getFiducialId();
    }

    @Override
    public Transform3d getCameraToTarget() {
      return target.getBestCameraToTarget();
    }

    @Override
    public double getPoseAmbiguity() {
      return target.getPoseAmbiguity();
    }

    AprilTagPhotonTarget(PhotonTrackedTarget target) {
      super(target);
    }
  }
}
