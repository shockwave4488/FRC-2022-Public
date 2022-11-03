package frc.robot.commands.c2022.defaults;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.c2022.Indexer;
import frc.robot.subsystems.c2022.Indexer.IndexerState;

public class DefaultIndexerLoad extends CommandBase {
  private final Indexer indexer;
  private final Indexer.StateSupplier indexerStates;
  private IndexerState state;

  public DefaultIndexerLoad(Indexer indexer) {
    this.indexer = indexer;
    indexerStates = indexer.getIndexerStates();
    addRequirements(indexer);
  }

  @Override
  public void initialize() {
    state = detectState();
    indexer.recordState(state);
  }

  @Override
  public void execute() {
    if (indexerStates.getFlywheelBeamBreak()) {
      state = IndexerState.TwoBalls;
    }
    switch (state) {
      case NoBalls:
        state = waitForBalls();
        break;
      case LoadingFirst:
        state = loadingFirst();
        break;
      case OneBall:
        state = loadSecond();
        break;
      case LoadingToFlywheel:
        state = loadToFlywheel();
        break;
      case TwoBalls:
        state = loadToFlywheel();
        break;
      default:
        state = IndexerState.TwoBalls;
        break;
        // NEED to add logging mechanism and tell driver through smart dashboard to manual over-ride
    }

    indexer.recordState(state);

    SmartDashboard.putString("Indexer State", state.toString());
  }

  private IndexerState waitForBalls() {
    indexer.spinHold();
    if (indexerStates.getEntranceBeamBreak()) {
      return IndexerState.LoadingFirst;
    }
    return IndexerState.NoBalls;
  }

  // Knowing we have no balls, spin conveyor until Middle BB broken.
  private IndexerState loadingFirst() {
    if (!indexerStates.getMiddleBeamBreak()) {
      indexer.spinForward();
      return IndexerState.LoadingFirst;
    } else {
      return IndexerState.OneBall;
    }
  }

  // Knowing we have 1 ball at Middle BB, stay stopped until Entrance BB broken then change state to
  // 2 balls
  private IndexerState loadSecond() {
    if (indexerStates.getEntranceBeamBreak()) {
      indexer.spinForward();
      return IndexerState.LoadingToFlywheel;
    } else {
      indexer.spinHold();
      return IndexerState.OneBall;
    }
  }

  // Knowing we have 2 balls, load till the first one is at the flywheel BB then stop
  private IndexerState loadToFlywheel() {
    if (!indexerStates.getFlywheelBeamBreak()) {
      indexer.spinForward();
      return IndexerState.LoadingToFlywheel;
    } else {
      indexer.spinHold();
      return IndexerState.TwoBalls;
    }
  }

  // Detect the state whenever this command is started
  private IndexerState detectState() {
    boolean entranceBB = indexerStates.getEntranceBeamBreak();
    boolean middleBB = indexerStates.getMiddleBeamBreak();
    boolean flywheelBB = indexerStates.getFlywheelBeamBreak();

    if (flywheelBB) {
      return IndexerState.TwoBalls;
    } else if (middleBB) {
      if (entranceBB) {
        return IndexerState.LoadingToFlywheel;
      } else {
        return IndexerState.OneBall;
      }
    } else if (entranceBB) {
      return IndexerState.LoadingFirst;
    } else {
      return IndexerState.NoBalls;
    }
  }

  @Override
  public void end(boolean interrupted) {
    indexer.spinHold();
  }
}
