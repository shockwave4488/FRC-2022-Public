package frc.robot.commands.c2022.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.c2022.indexer.IndexerAdvance2;
import frc.robot.subsystems.c2022.Indexer;
import frc.robot.subsystems.c2022.Shooter;
import frc.robot.subsystems.c2022.SmartPCM;

public class LaunchSetShot extends SequentialCommandGroup {
  private final SmartPCM compressor;

  public LaunchSetShot(
      Shooter shooter,
      Indexer conveyor,
      SmartPCM compressor,
      double setRPM,
      double desiredHoodPos) {
    this.compressor = compressor;

    addRequirements(shooter);
    addRequirements(conveyor);
    addRequirements(compressor);

    addCommands(
        new InstantCommand(() -> shooter.setRPM(setRPM)),
        new InstantCommand(() -> shooter.setHoodPosition(desiredHoodPos)),
        new WaitUntilCommand(shooter::isReady),
        new IndexerAdvance2(conveyor));
  }

  @Override
  public void initialize() {
    super.initialize();

    compressor.stopCompressor();
  }

  public void end(boolean interrupted) {
    super.end(interrupted);
    compressor.startCompressor();
  }
}
