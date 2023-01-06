package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.autonomous.modes.AutonomousChooser.AutoPIDControllerContainer;
import frc.robot.subsystems.drive.SwerveDrive;
import java.util.Arrays;
import java.util.function.Supplier;

public class SwerveDriveToPosition extends CommandBase {
  private static Trajectory constructTrajectory(
      Pose2d goalPose, Pose2d curPose, TrajectoryConfig config, Translation2d[] interiorWaypoints) {
    return TrajectoryGenerator.generateTrajectory(
        curPose, Arrays.asList(interiorWaypoints), goalPose, config);
  }

  private static Rotation2d getFinalTrajectoryRotation(Trajectory trajectory) {
    return trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation();
  }

  private final SwerveDrive swerve;
  private final AutoPIDControllerContainer pidControllers;
  private final Supplier<Trajectory> trajectory;
  private final Supplier<Rotation2d> desiredRotation;
  private boolean finalRotationMode = false;

  private Rotation2d finalRotation;
  private SwerveControllerCommand currentSwerveControllerCommand;
  private Trajectory currentTrajectory;

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
      Supplier<TrajectoryConfig> config,
      Supplier<Pose2d> goalPose,
      Supplier<Rotation2d> desiredRotation,
      Supplier<Translation2d[]> interiorWaypoints) {
    this(
        swerve,
        pidControllers,
        () -> {
          TrajectoryConfig configInst = config.get();
          // TODO: Figure out how to decide when to reverse the trajectory config
          configInst.setReversed(false);
          Trajectory trajectory =
              constructTrajectory(
                  goalPose.get(), swerve.getOdometry(), configInst, interiorWaypoints.get());
          // SmartDashboard.putString("Trajectory start", swerve.getOdometry().toString());
          // SmartDashboard.putString("Trajectory end", goalPose.get().toString());
          // SmartDashboard.putString("Generated trajectory", trajectory.toString());
          return trajectory;
        },
        desiredRotation);
  }

  public SwerveDriveToPosition(
      SwerveDrive swerve,
      AutoPIDControllerContainer pidControllers,
      Supplier<TrajectoryConfig> config,
      Supplier<Pose2d> goalPose,
      Supplier<Translation2d[]> interiorWaypoints) {
    this(
        swerve,
        pidControllers,
        config,
        goalPose,
        () -> goalPose.get().getRotation(),
        interiorWaypoints);
    finalRotationMode = true;
  }

  public Trajectory getTrajectory() {
    return currentTrajectory;
  }

  @Override
  public void initialize() {
    currentTrajectory = trajectory.get();
    if (finalRotationMode) finalRotation = desiredRotation.get();
    currentSwerveControllerCommand =
        new SwerveControllerCommand(
            currentTrajectory,
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
