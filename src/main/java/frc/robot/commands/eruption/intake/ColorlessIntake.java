package frc.robot.commands.eruption.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.eruption.Intake;
import frc.robot.subsystems.eruption.Intake.RollerState;

public class ColorlessIntake extends CommandBase {
  // Backup intake if color intake fails

  private final Intake intake;

  public ColorlessIntake(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.intakeOut();
  }

  @Override
  public void execute() {
    intake.setTopRollerState(RollerState.ForwardFull);
    intake.setBottomRollerState(RollerState.ForwardFull);
  }
}
