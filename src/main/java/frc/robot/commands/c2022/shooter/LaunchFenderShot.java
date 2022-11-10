package frc.robot.commands.c2022.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.c2022.indexer.IndexerAdvanceForFenderShot;
import frc.robot.subsystems.c2022.Indexer;
import frc.robot.subsystems.c2022.Shooter;
import frc.robot.subsystems.c2022.SmartPCM;

public class LaunchFenderShot extends SequentialCommandGroup {
  private final Shooter shooter;
  private final SmartPCM compressor;

  public LaunchFenderShot(
      Shooter shooter,
      Indexer conveyor,
      SmartPCM compressor,
      double setRPM,
      double desiredHoodPos) {
    this.shooter = shooter;
    this.compressor = compressor;

    addRequirements(shooter);
    addRequirements(conveyor);
    addRequirements(compressor);

    addCommands(
        new InstantCommand(() -> shooter.setRPM(setRPM)),
        new InstantCommand(() -> shooter.setHoodPosition(desiredHoodPos)),
        new WaitUntilCommand(shooter::isReady),
        new IndexerAdvanceForFenderShot(conveyor)
            .alongWith(new InstantCommand(() -> shooter.setHoodPosition(desiredHoodPos))));
  }

  @Override
  public void initialize() {
    super.initialize();

    compressor.stopCompressor();
  }

  public void end(boolean interrupted) {
    super.end(interrupted);
    compressor.startCompressor();
    shooter.stop();
  }
}
