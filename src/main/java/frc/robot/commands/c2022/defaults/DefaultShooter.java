package frc.robot.commands.c2022.defaults;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.sensors.vision.Limelight;
import frc.robot.commands.c2022.shooter.SpinFlywheel;
import frc.robot.subsystems.c2022.Indexer;
import frc.robot.subsystems.c2022.Shooter;
import java.util.function.Supplier;

public class DefaultShooter extends SequentialCommandGroup {
  private static final double COAST_RPM = 0;
  private static final boolean ESTIMATE = true; // inaccurate but could be better than nothing

  public DefaultShooter(
      Shooter shooter, Limelight limelight, Indexer indexer, Supplier<Pose2d> pose) {
    addRequirements(shooter);

    addCommands(
        new InstantCommand(() -> shooter.setRPM(COAST_RPM)),
        new SpinFlywheel(shooter, limelight, indexer, pose, ESTIMATE, false, false));
    // Even though the above appears that it will spin the shooter if we see the target, it won't,
    // it never actuals tells the shooter to spin because interpolateShooter is false.
  }
}
