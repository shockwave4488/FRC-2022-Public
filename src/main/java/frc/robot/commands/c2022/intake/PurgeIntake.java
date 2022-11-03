package frc.robot.commands.c2022.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.c2022.Intake;
import frc.robot.subsystems.c2022.Intake.RollerState;

public class PurgeIntake extends CommandBase {
  private final Intake intake;

  public PurgeIntake(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intake.intakeOut();
    intake.setTopRollerState(RollerState.Off);
    intake.setBottomRollerState(RollerState.ForwardMedium);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
