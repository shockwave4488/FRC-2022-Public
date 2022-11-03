package frc.robot.commands.c2022.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.c2022.Intake;
import frc.robot.subsystems.c2022.Intake.RollerState;

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
