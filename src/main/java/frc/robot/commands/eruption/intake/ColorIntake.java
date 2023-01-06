package frc.robot.commands.eruption.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.eruption.Indexer;
import frc.robot.subsystems.eruption.Indexer.IndexerState;
import frc.robot.subsystems.eruption.Intake;
import frc.robot.subsystems.eruption.Intake.RollerState;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class ColorIntake extends CommandBase {
  private final Intake intake;
  private final BooleanSupplier hasTwoBalls;
  private final boolean hardPurge;

  public ColorIntake(Intake intake, Indexer.StateSupplier indexerStates, boolean hardPurge) {
    this.intake = intake;
    this.hardPurge = hardPurge;
    addRequirements(intake);

    Supplier<IndexerState> curIndexerState = () -> indexerStates.getState();

    hasTwoBalls =
        () ->
            (curIndexerState.get() == IndexerState.LoadingToFlywheel
                || curIndexerState.get() == IndexerState.TwoBalls);
  }

  @Override
  public void execute() {
    intake.intakeOut();
    if (intake.hasCargo()) {
      if (intake.hasCorrectColor()) {
        intake.setTopRollerState(RollerState.ForwardFull);
        intake.setBottomRollerState(RollerState.ForwardFull);
      } else if (!intake.hasCorrectColor()) {
        if (!hardPurge) {
          intake.setTopRollerState(RollerState.ReverseCrawl);
          intake.setBottomRollerState(RollerState.ForwardMedium);
        } else {
          intake.setTopRollerState(RollerState.ReverseFull);
          intake.setBottomRollerState(RollerState.ForwardFull);
        }
      }
    } else {
      intake.setTopRollerState(RollerState.Off);
      intake.setBottomRollerState(RollerState.ForwardFull);
    }

    SmartDashboard.putBoolean("Color intake has two balls", hasTwoBalls.getAsBoolean());
  }

  @Override
  public boolean isFinished() {
    return hasTwoBalls.getAsBoolean();
  }
}
