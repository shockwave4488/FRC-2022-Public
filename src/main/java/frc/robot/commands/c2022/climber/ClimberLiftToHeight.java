package frc.robot.commands.c2022.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.c2022.Climber;

public class ClimberLiftToHeight extends CommandBase {
  private final Climber climber;
  private final int desiredTicks;
  private final int minDoneTicks;
  private final int doneTickRange;

  public ClimberLiftToHeight(
      Climber climber, int desiredTicks, int minDoneTicks, int doneTickRange) {
    this.climber = climber;
    this.desiredTicks = desiredTicks;
    this.minDoneTicks = minDoneTicks;
    this.doneTickRange = doneTickRange;
  }

  /**
   * Use this constructor to set the climber to a certain height without caring about the climber
   * subsystem's isStable method. Since this constructor doesn't care about isStable, this command
   * will instantly end after setting the climber's desired position.
   *
   * @param climber The climber subsystem object
   * @param desiredTicks The desired height of the climber in motor ticks
   */
  public ClimberLiftToHeight(Climber climber, int desiredTicks) {
    this(climber, desiredTicks, 0, 999999);
  }

  public ClimberLiftToHeight(Climber climber) {
    this(climber, climber.getUpperLimit());
  }

  @Override
  public void initialize() {
    climber.setClimberPosition(desiredTicks, minDoneTicks, doneTickRange);
  }

  @Override
  public boolean isFinished() {
    return climber.isStable();
  }
}
