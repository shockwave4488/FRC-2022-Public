package frc.robot.commands.c2022.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.c2022.Indexer;
import frc.robot.subsystems.c2022.Intake;
import frc.robot.subsystems.c2022.Intake.RollerState;
import frc.robot.subsystems.c2022.Shooter;

public class PurgeBack extends CommandBase {
  private final Intake intake;
  private final Indexer indexer;
  private final Shooter shooter;

  public PurgeBack(Intake intake, Indexer indexer, Shooter shooter) {
    this.intake = intake;
    this.indexer = indexer;
    this.shooter = shooter;
    addRequirements(intake, indexer, shooter);
  }

  @Override
  public void execute() {
    intake.intakeOut();
    intake.setBottomRollerState(RollerState.ReverseMedium);
    intake.setTopRollerState(RollerState.ReverseMedium);
    indexer.spinBackwards();
    shooter.setRPM(0);
  }
}
