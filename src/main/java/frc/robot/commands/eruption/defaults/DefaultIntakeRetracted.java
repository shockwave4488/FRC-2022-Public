package frc.robot.commands.eruption.defaults;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.eruption.Intake;

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
