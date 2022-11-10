package frc.robot.commands.c2022.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.c2022.Indexer;
import frc.robot.subsystems.c2022.Indexer.IndexerState;
import frc.robot.subsystems.c2022.Shooter;

public class PurgeForward extends CommandBase {
  private final Indexer indexer;
  private final Shooter shooter;
  private double startTime;
  private static final double WAIT_TIME = 0.5;

  public PurgeForward(Indexer indexer, Shooter shooter) {
    this.indexer = indexer;
    this.shooter = shooter;

    addRequirements(indexer, shooter);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    indexer.recordState(IndexerState.Flushing);
  }

  @Override
  public void execute() {
    shooter.setRPM(1000);
    shooter.setHoodPosition(55);
    if (Timer.getFPGATimestamp() > startTime + WAIT_TIME) {
      indexer.spinForward();
    }
  }
}
