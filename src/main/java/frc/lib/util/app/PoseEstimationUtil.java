package frc.lib.util.app;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.sensors.vision.VisionCamera;
import frc.lib.sensors.vision.VisionTargets.Pose3dTarget;
import frc.robot.subsystems.drive.SwerveDrive;

public class PoseEstimationUtil {
  private PoseEstimationUtil() {}

  /**
   * Estimates range to a target using the target's elevation, and only 2D vision information. This
   * method can produce more stable results than SolvePnP when well tuned, if the full 6d robot pose
   * is not required. Note that this method requires the camera to have 0 roll (not be skewed
   * clockwise or CCW relative to the floor), and for there to exist a height differential between
   * goal and camera. The larger this differential, the more accurate the distance estimate will be.
   */
  public static double calculateDistanceToTarget(
      double cameraHeightMeters,
      double targetHeightMeters,
      Rotation2d cameraPitch,
      Rotation2d targetPitch,
      Rotation2d targetYaw) {
    return (targetHeightMeters - cameraHeightMeters)
        / (cameraPitch.plus(targetPitch).getTan() * targetYaw.getCos());
  }

  /**
   * Returns the pose of the robot center from tranform information between a reference point (such
   * as a vision target) to the camera.
   */
  public static Pose2d getRobotPoseFromTransform(
      Pose3d referencePose, Transform3d cameraToReference, Transform3d robotCenterToCamera) {
    return referencePose
        .transformBy(cameraToReference.inverse())
        .transformBy(robotCenterToCamera.inverse())
        .toPose2d();
  }

  /**
   * Returns the pose of a reference point (such as a vision target) from tranform information
   * between the camera and the reference point.
   */
  public static Pose3d getTargetPoseFromTransform(
      Pose2d robotPose, Transform3d cameraToTarget, Transform3d robotCenterToCamera) {
    return new Pose3d(robotPose).transformBy(robotCenterToCamera).transformBy(cameraToTarget);
  }

  /**
   * Compute and consume a vision measurement yielding a 3D transformation (such as solvePnP for
   * AprilTags).
   *
   * @param swerve Drive subsystem using the estimated robot position.
   * @param targetPose Pose of the vision target on the field (likely a static position).
   * @param targetToCamera Transform from the target pose to the camera pose, likely retrieved from
   *     a PhotonTrackedTarget.
   * @param cameraToRobotCenter Transform from the center of the robot to the mounted location of
   *     the camera. In this case (not always), the Z-axis is irrelevant because the final pose will
   *     be projected to two dimensions.
   */
  public static void consumePoseFromTransform(
      SwerveDrive swerve,
      Pose3d targetPose,
      Transform3d cameraToTarget,
      Transform3d robotCenterToCamera) {
    Pose2d robotCenterPose =
        getRobotPoseFromTransform(targetPose, cameraToTarget, robotCenterToCamera);
    swerve.consumeVisionEstimate(robotCenterPose);
  }

  public static void consumePoseFromBestTransform(
      SwerveDrive swerve, Pose3d targetPose, VisionCamera<? extends Pose3dTarget> camera) {
    consumePoseFromTransform(
        swerve,
        targetPose,
        camera.getBestTarget().get().getCameraToTarget(),
        camera.getCameraPositionConsts().robotCenterToCamera);
  }

  /**
   * This method will calculate an estimated robot position based on a given translational distance
   * to a target whose position is provided, and the angle to the target. Only call this method if
   * you know your translational distance is correct.
   *
   * @param translationalDistance Translational distance from the target in meters
   * @see {@link #calculateDistanceToTarget(double, double, Rotation2d, Rotation2d, Rotation2d)}
   */
  public static void consumePoseFrom2d(
      SwerveDrive swerve,
      Translation2d targetPose,
      double translationalDistance,
      Rotation2d angleToTarget) {
    // The robot's quadrant on the field does not matter when doing the following calculations.
    Translation2d relativeTransform =
        new Translation2d(
            -angleToTarget.getCos() * translationalDistance,
            -angleToTarget.getSin() * translationalDistance);
    Translation2d visionEstimatedPos = targetPose.plus(relativeTransform);

    swerve.consumeVisionEstimate(
        new Pose2d(visionEstimatedPos, swerve.getOdometry().getRotation()));
  }
}
