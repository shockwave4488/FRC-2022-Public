package frc.robot.commands.eruption.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.sensors.NavX;
import frc.lib.sensors.vision.Limelight;
import frc.robot.commands.drive.SwerveDriveWithHeading;
import frc.robot.commands.eruption.drive.SwerveTurnToHub;
import frc.robot.commands.eruption.drive.VisionAlignToTarget;
import frc.robot.commands.eruption.indexer.IndexerAdvance2;
import frc.robot.subsystems.SmartPCM;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.eruption.Indexer;
import frc.robot.subsystems.eruption.Shooter;
import java.util.function.Supplier;

public class CalculatedShot extends ParallelRaceGroup {
  private static final int MIN_ALIGN_CYCLES = 8;

  public CalculatedShot(
      Shooter shooter,
      Indexer conveyor,
      SmartPCM compressor,
      SwerveDrive swerve,
      NavX gyro,
      Supplier<double[]> driveValues,
      double driveMultiplier,
      double rotationMultiplier,
      Limelight limelight,
      SwerveTurnToHub turnToHubCommand,
      SwerveDriveWithHeading headingCommand) {

    VisionAlignToTarget alignToTarget =
        new VisionAlignToTarget(
            swerve,
            limelight,
            gyro,
            driveMultiplier,
            rotationMultiplier,
            driveValues,
            MIN_ALIGN_CYCLES,
            false);

    SpinFlywheel spinFlywheel =
        new SpinFlywheel(
            shooter,
            limelight,
            conveyor.getIndexerStates()::getFlywheelBeamBreak,
            swerve::getOdometry,
            true,
            true,
            false);

    InstantCommand stopCompressor =
        new InstantCommand(() -> compressor.stopCompressor(), compressor);

    addCommands(
        spinFlywheel.alongWith(stopCompressor),
        turnToHubCommand
            .andThen(headingCommand)
            .andThen(
                new ParallelRaceGroup(
                    alignToTarget,
                    new WaitUntilCommand(() -> alignToTarget.readyToShoot() && shooter.isReady())
                        .andThen(new IndexerAdvance2(conveyor)))));
  }
}
