package frc.robot.commands.eruption.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.eruption.Indexer;

public class IndexerDeactivated extends CommandBase {
  private final Indexer indexer;

  public IndexerDeactivated(Indexer indexer) {
    this.indexer = indexer;
    addRequirements(indexer);
  }

  @Override
  public void execute() {
    indexer.spinHold();
  }
}
