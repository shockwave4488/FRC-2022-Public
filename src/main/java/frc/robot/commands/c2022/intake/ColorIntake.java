package frc.robot.commands.c2022.intake;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.c2022.Indexer;
import frc.robot.subsystems.c2022.Indexer.IndexerState;
import frc.robot.subsystems.c2022.Intake;
import frc.robot.subsystems.c2022.Intake.RollerState;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class ColorIntake extends SequentialCommandGroup {
  // Purge mode could potentially be used over retraction when we're able to constantly drive and
  // shoot?

  private static final double RETRACT_PURGE_TIME = 0.4;
  private boolean hardPurge = false;

  public ColorIntake(Intake intake, Indexer.StateSupplier indexerStates, boolean retractWhenFull) {
    addRequirements(intake);

    Supplier<IndexerState> curIndexerState = () -> indexerStates.getState();
    BooleanSupplier hasTwoBalls =
        () ->
            (curIndexerState.get() == IndexerState.LoadingToFlywheel
                || curIndexerState.get() == IndexerState.TwoBalls);

    SequentialCommandGroup idle =
        new InstantCommand(() -> intake.setTopRollerState(RollerState.Off))
            .andThen(
                new InstantCommand(() -> intake.setBottomRollerState(RollerState.ForwardFull)));

    InstantCommand intakeBall =
        new InstantCommand(() -> intake.setTopRollerState(RollerState.ForwardFull));

    SequentialCommandGroup intakeBallUntil =
        intakeBall.andThen(
            new WaitUntilCommand(() -> (!intake.hasCargo() || !intake.hasCorrectColor())));

    Supplier<SequentialCommandGroup> purge =
        () ->
            new InstantCommand(() -> intake.setTopRollerState(RollerState.ReverseCrawl))
                .andThen(
                    new InstantCommand(
                        () -> intake.setBottomRollerState(RollerState.ForwardMedium)));
    InstantCommand hardPurgeCommand =
        new InstantCommand(
            () -> intake.setTopRollerState(RollerState.ReverseFull)); // Bottom roller at full
    SequentialCommandGroup purgeUntil =
        new ConditionalCommand(hardPurgeCommand, purge.get(), () -> hardPurge)
            .andThen(new WaitUntilCommand(() -> !intake.hasCargo() || intake.hasCorrectColor()));

    ConditionalCommand intakeDirectionChoice =
        new ConditionalCommand(intakeBallUntil, purgeUntil, intake::hasCorrectColor);

    WaitUntilCommand waitForIntakeTrigger = new WaitUntilCommand(() -> intake.hasCargo());

    SequentialCommandGroup retractWhilePurging =
        new SequentialCommandGroup(
            new InstantCommand(() -> intake.intakeIn()),
            new InstantCommand(() -> intake.setTopRollerState(RollerState.ReverseCrawl)),
            new InstantCommand(() -> intake.setBottomRollerState(RollerState.ReverseCrawl)),
            new WaitCommand(RETRACT_PURGE_TIME),
            new InstantCommand(() -> intake.onStop()));

    SequentialCommandGroup startIntakeChoice;
    if (!retractWhenFull) {
      startIntakeChoice =
          new InstantCommand(() -> intake.intakeOut())
              .andThen(
                  new ConditionalCommand(
                      purge
                          .get()
                          .andThen(
                              new WaitUntilCommand(
                                  () -> curIndexerState.get() == IndexerState.Flushing)),
                      new InstantCommand(),
                      hasTwoBalls));
    } else {
      startIntakeChoice =
          new ConditionalCommand(
                  new ConditionalCommand(
                          retractWhilePurging, new InstantCommand(), intake::getIntakeOut)
                      .andThen(
                          new WaitUntilCommand(
                              () -> curIndexerState.get() == IndexerState.Flushing)),
                  new InstantCommand(),
                  hasTwoBalls)
              .andThen(new InstantCommand(() -> intake.intakeOut()));
    }

    addCommands(
        startIntakeChoice,
        new SequentialCommandGroup(idle, waitForIntakeTrigger, intakeDirectionChoice)
            .withInterrupt(hasTwoBalls));
  }

  public ColorIntake(
      Intake intake,
      Indexer.StateSupplier indexerStates,
      boolean retractWhenFull,
      boolean hardPurge) {
    this(intake, indexerStates, retractWhenFull);
    this.hardPurge = hardPurge;
  }
}
