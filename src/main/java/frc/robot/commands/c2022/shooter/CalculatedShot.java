package frc.robot.commands.c2022.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.sensors.Limelight;
import frc.lib.sensors.NavX;
import frc.robot.Constants;
import frc.robot.commands.c2022.drive.SwerveDriveWithHeading;
import frc.robot.commands.c2022.drive.SwerveTurnToHUB;
import frc.robot.commands.c2022.drive.VisionAlignToTarget;
import frc.robot.commands.c2022.indexer.IndexerAdvance2;
import frc.robot.subsystems.c2022.Indexer;
import frc.robot.subsystems.c2022.Shooter;
import frc.robot.subsystems.c2022.SmartPCM;
import frc.robot.subsystems.drive.SwerveDrive;
import java.util.function.Supplier;

public class CalculatedShot extends SequentialCommandGroup {
  private final Shooter shooter;
  private final SmartPCM compressor;
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
      SwerveTurnToHUB turnToHUBCommand,
      SwerveDriveWithHeading headingCommand) {
    this.shooter = shooter;
    this.compressor = compressor;

    addRequirements(shooter);
    addRequirements(conveyor);
    addRequirements(compressor);
    addRequirements(swerve);
    addRequirements(limelight);

    addCommands(
        turnToHUBCommand,
        headingCommand,
        new VisionAlignToTarget(
                swerve,
                limelight,
                gyro,
                driveMultiplier,
                rotationMultiplier,
                driveValues,
                MIN_ALIGN_CYCLES,
                true)
            .alongWith(
                new SpinFlywheel(
                    shooter, limelight, conveyor, () -> swerve.getOdometry(), true, true, true)),
        new IndexerAdvance2(conveyor)
            .deadlineWith(
                new SpinFlywheel(
                    shooter, limelight, conveyor, () -> swerve.getOdometry(), false, true, false))
            .deadlineWith(
                new VisionAlignToTarget(
                    swerve,
                    limelight,
                    gyro,
                    driveMultiplier,
                    rotationMultiplier,
                    driveValues,
                    10,
                    false)));
  }

  @Override
  public void initialize() {
    super.initialize();

    SmartDashboard.putBoolean("AutoShot", true);
    shooter.setRPM(Constants.ShooterConstants.BACK_OF_TARMAC_RPM);
    compressor.stopCompressor();
  }

  public void end(boolean interrupted) {
    super.end(interrupted);

    SmartDashboard.putBoolean("AutoShot", false);
    compressor.startCompressor();
  }
}
