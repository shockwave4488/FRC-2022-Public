package frc.robot.commands.c2022.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.c2022.Indexer;

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
