package frc.robot.commands.c2022.defaults;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.c2022.Intake;

public class DefaultIntakeRetracted extends CommandBase {
  private final Intake intake;

  public DefaultIntakeRetracted(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    intake.onStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
