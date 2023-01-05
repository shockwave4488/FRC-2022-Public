package frc.robot.commands.c2022.drive;

import edu.umd.cs.findbugs.annotations.SuppressFBWarnings;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.autonomous.modes.AutonomousChooser.AutoPIDControllerContainer;
import frc.robot.subsystems.drive.SwerveDrive;
import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class SwerveDriveToPosition extends CommandBase {
  static {
    SmartDashboard.putNumber("Autonomous max speed", 1.5);
  }

  private static Trajectory constructTrajectory(
      Pose2d goalPose,
      Pose2d curPose,
      SwerveDriveKinematics kinematics,
      Translation2d[] interiorWaypoints,
      boolean reversed) {
    TrajectoryConfig config =
        new TrajectoryConfig(
                SmartDashboard.getNumber("Autonomous max speed", 1.5),
                DriveTrainConstants.SWERVE_DRIVE_MAX_ACCEL)
            .setKinematics(kinematics);
    config.setReversed(reversed);
    return TrajectoryGenerator.generateTrajectory(
        curPose, Arrays.asList(interiorWaypoints), goalPose, config);
  }

  private static Rotation2d getFinalTrajectoryRotation(Trajectory trajectory) {
    return trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation();
  }

  @SuppressFBWarnings("EI_EXPOSE_STATIC_REP2")
  public static void setField2d(Field2d field2d) {
    field = field2d;
  }

  private static Field2d field;

  private final SwerveDrive swerve;
  private final AutoPIDControllerContainer pidControllers;
  private final Supplier<Trajectory> trajectory;
  private final Supplier<Rotation2d> desiredRotation;
  private boolean finalRotationMode = false;

  private Rotation2d finalRotation;
  private SwerveControllerCommand currentSwerveControllerCommand;

  public SwerveDriveToPosition(
      SwerveDrive swerve,
      AutoPIDControllerContainer pidControllers,
      Supplier<Trajectory> trajectory,
      Supplier<Rotation2d> desiredRotation) {
    this.swerve = swerve;
    this.pidControllers = pidControllers;
    this.trajectory = trajectory;
    this.desiredRotation = desiredRotation;
    addRequirements(swerve);
  }

  public SwerveDriveToPosition(
      SwerveDrive swerve,
      AutoPIDControllerContainer pidControllers,
      Supplier<Trajectory> trajectory) {
    this(swerve, pidControllers, trajectory, () -> getFinalTrajectoryRotation(trajectory.get()));
    finalRotationMode = true;
  }

  public SwerveDriveToPosition(
      SwerveDrive swerve,
      AutoPIDControllerContainer pidControllers,
      Supplier<Pose2d> goalPose,
      Supplier<Rotation2d> desiredRotation,
      BooleanSupplier reversed,
      Supplier<Translation2d[]> interiorWaypoints) {
    this(
        swerve,
        pidControllers,
        () -> {
          Trajectory trajectory =
              constructTrajectory(
                  goalPose.get(),
                  swerve.getOdometry(),
                  swerve.getKinematics(),
                  interiorWaypoints.get(),
                  reversed.getAsBoolean());
          if (field != null) {
            field.getObject("Trajectory").setTrajectory(trajectory);
          }
          SmartDashboard.putString("Trajectory start", swerve.getOdometry().toString());
          SmartDashboard.putString("Trajectory end", goalPose.get().toString());
          SmartDashboard.putString("Generated trajectory", trajectory.toString());
          return trajectory;
        },
        desiredRotation);
  }

  public SwerveDriveToPosition(
      SwerveDrive swerve,
      AutoPIDControllerContainer pidControllers,
      Supplier<Pose2d> goalPose,
      BooleanSupplier reversed,
      Supplier<Translation2d[]> interiorWaypoints) {
    this(
        swerve,
        pidControllers,
        goalPose,
        () -> goalPose.get().getRotation(),
        reversed,
        interiorWaypoints);
    finalRotationMode = true;
  }

  @Override
  public void initialize() {
    if (finalRotationMode) finalRotation = desiredRotation.get();
    currentSwerveControllerCommand =
        new SwerveControllerCommand(
            trajectory.get(),
            swerve::getOdometry,
            swerve.getKinematics(),
            pidControllers.xPidController,
            pidControllers.yPidController,
            pidControllers.thetaPidController,
            (finalRotationMode) ? () -> finalRotation : desiredRotation,
            swerve::setModuleStates,
            swerve);
    currentSwerveControllerCommand.initialize();
  }

  @Override
  public void execute() {
    if (currentSwerveControllerCommand != null) {
      currentSwerveControllerCommand.execute();
    }
  }

  @Override
  public boolean isFinished() {
    return (currentSwerveControllerCommand != null)
        ? currentSwerveControllerCommand.isFinished()
        : false;
  }

  @Override
  public void end(boolean interrupted) {
    if (currentSwerveControllerCommand != null) {
      currentSwerveControllerCommand.end(interrupted);
    }
  }
}
