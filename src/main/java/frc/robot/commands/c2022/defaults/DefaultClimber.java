package frc.robot.commands.c2022.defaults;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.c2022.Climber;

public class DefaultClimber extends CommandBase {
  private final Climber climber;

  public DefaultClimber(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.setClimberPosition(0);
  }
}
