package frc.robot.commands.eruption.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.eruption.Indexer;

public class IndexerAdvanceForFenderShot extends CommandBase {
  private final Indexer indexer;
  private double startTime;
  private static final double RUN_TIME = 2;

  public IndexerAdvanceForFenderShot(Indexer indexer) {
    this.indexer = indexer;
    addRequirements(indexer);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    indexer.spinToFenderShoot();
  }

  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() > startTime + RUN_TIME);
  }

  @Override
  public void end(boolean interrupted) {
    indexer.spinHold();
  }
}
