package frc.lib.util.app;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

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
    // Use ComputerVisionUtil with 2023 WPILIb
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
    // With 2023 beta, there is a Pose3d(Pose2d) constructor.
    return new Pose3d(
            robotPose.getTranslation().getX(),
            robotPose.getTranslation().getY(),
            0,
            new Rotation3d(0, 0, robotPose.getRotation().getRadians()))
        .transformBy(robotCenterToCamera)
        .transformBy(cameraToTarget);
  }
}
