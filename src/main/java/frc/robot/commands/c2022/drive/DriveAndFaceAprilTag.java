package frc.robot.commands.c2022.drive;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.sensors.vision.VisionTargets.AprilTagTarget;
import frc.robot.autonomous.modes.AutonomousChooser.AutoPIDControllerContainer;
import frc.robot.subsystems.drive.SwerveDrive;
import java.util.function.Supplier;

/** Command for autonomously driving to a place relative to the position of a still AprilTag */
public class DriveAndFaceAprilTag extends SwerveDriveToPosition {
  private static Pose2d getDesiredRobotPose(Pose2d tagPose, Transform2d adjustment) {
    Translation2d desiredRobotPos = tagPose.transformBy(adjustment).getTranslation();
    Translation2d diff = tagPose.getTranslation().minus(desiredRobotPos);
    // Translation2d.getAngle() can be used with 2023 beta instead
    Rotation2d angleToTag = new Rotation2d(diff.getX(), diff.getY());
    return new Pose2d(desiredRobotPos, angleToTag.plus(adjustment.getRotation()));
  }

  /**
   * This constructor works backwards from the known robot pose to find the coords of the tag and
   * drive there.
   *
   * @param adjustment Transform (from the perspective of the tag) to be added to the calculated tag
   *     coordinates. For example, "Transform2d(Translation2d(2, 0), Rotation2d(pi/4))" would result
   *     in the robot center stopping 2 meters in front of the tag, pointing 45 degrees to the left
   *     of the tag.
   */
  public DriveAndFaceAprilTag(
      SwerveDrive swerve,
      AutoPIDControllerContainer pidControllers,
      Supplier<? extends AprilTagTarget> aprilTagTarget,
      Transform3d robotCenterToCamera,
      Supplier<Transform2d> adjustment) {
    this(
        swerve,
        pidControllers,
        () -> aprilTagTarget.get().toAprilTag(swerve.getOdometry(), robotCenterToCamera),
        adjustment);
  }

  /**
   * @param tag AprilTag to drive to (stored pose must be up-to-date)
   * @param adjustment Transform (from the perspective of the tag) to be added to the calculated tag
   *     coordinates. For example, "Transform2d(Translation2d(0, 2), Rotation2d(pi/4))" would result
   *     in the robot center stopping 2 meters in front of the tag, pointing 45 degrees to the left
   *     of the tag.
   */
  public DriveAndFaceAprilTag(
      SwerveDrive swerve,
      AutoPIDControllerContainer pidControllers,
      Supplier<AprilTag> tag,
      Supplier<Transform2d> adjustment) {
    // TODO: Figure out how to decide when to reverse the trajectory config
    super(
        swerve,
        pidControllers,
        () -> getDesiredRobotPose(tag.get().pose.toPose2d(), adjustment.get()),
        () -> false,
        () -> new Translation2d[0]);
  }
}
