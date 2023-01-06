package frc.robot.commands.eruption.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.eruption.Climber;

public class NoBallOnClimberArm extends CommandBase {
  private final Climber climber;
  private static final int MIN_COUNT_CYCLES = 10;
  private static final int ERROR_EPSILION = 5000;
  private static final int CLIMBER_UP_TICKS = 40000;

  public NoBallOnClimberArm(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.setClimberPosition(CLIMBER_UP_TICKS, MIN_COUNT_CYCLES, ERROR_EPSILION);
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
