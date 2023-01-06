package frc.robot.commands.eruption.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.eruption.Indexer;

public class IndexerAdvance2 extends CommandBase {
  private final Indexer indexer;
  private double startTime;
  private static final double RUN_TIME = 1;

  public IndexerAdvance2(Indexer indexer) {
    this.indexer = indexer;
    addRequirements(indexer);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    indexer.spinToShoot();
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
