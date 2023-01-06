package frc.robot.commands.eruption.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.eruption.Climber;
import java.util.function.BooleanSupplier;

public class ClimberLiftToHeight extends CommandBase {
  private final Climber climber;
  private final int desiredTicks;
  private final int minDoneTicks;
  private final int doneTickRange;
  // abort button exists so we can cancel this action if the arm up button is accidentally hit
  private final BooleanSupplier abort;

  public ClimberLiftToHeight(
      Climber climber,
      int desiredTicks,
      int minDoneTicks,
      int doneTickRange,
      BooleanSupplier abort) {
    this.climber = climber;
    this.desiredTicks = desiredTicks;
    this.minDoneTicks = minDoneTicks;
    this.doneTickRange = doneTickRange;
    this.abort = abort;
  }

  /**
   * Use this constructor to set the climber to a certain height without caring about the climber
   * subsystem's isStable method. Since this constructor doesn't care about isStable, this command
   * will instantly end after setting the climber's desired position.
   *
   * @param climber The climber subsystem object
   * @param desiredTicks The desired height of the climber in motor ticks
   */
  public ClimberLiftToHeight(Climber climber, int desiredTicks, BooleanSupplier abort) {
    this(climber, desiredTicks, 2, 3000, abort);
  }

  public ClimberLiftToHeight(Climber climber, BooleanSupplier abort) {
    this(climber, climber.getUpperLimit(), abort);
  }

  @Override
  public void initialize() {
    climber.setClimberPosition(desiredTicks, minDoneTicks, doneTickRange);
  }

  @Override
  public boolean isFinished() {
    return climber.isStable() || abort.getAsBoolean();
  }
}
