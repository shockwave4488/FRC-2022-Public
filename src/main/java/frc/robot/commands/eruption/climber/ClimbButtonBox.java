package frc.robot.commands.eruption.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.eruption.Climber;
import frc.robot.subsystems.eruption.Shooter;
import java.util.function.BooleanSupplier;

public class ClimbButtonBox extends CommandBase {
  private final Climber climber;
  private final Shooter shooter;
  private final BooleanSupplier moveUp;
  private final BooleanSupplier moveDown;
  private int desiredTicks;
  private static final int CLIMB_DESTRUCTION_TICKS = 20000;
  private static final int CLIMB_SPEED_ARM_DOWN = 3400; // 3000 -> 3100 -> 3400
  private static final int CLIMB_SPEED_ARM_UP = 5200; // was 2500
  private boolean armWasUp;

  /**
   * Control the climber with two buttons
   *
   * @param climber Climber subsystem
   * @param moveUp Hit this button to move the climber all the way up
   * @param moveDown Hit this button to slowly move the climber down
   */
  public ClimbButtonBox(
      Climber climber, Shooter shooter, BooleanSupplier moveUp, BooleanSupplier moveDown) {
    this.climber = climber;
    this.moveUp = moveUp;
    this.moveDown = moveDown;
    this.shooter = shooter;

    addRequirements(climber, shooter);
  }

  @Override
  public void initialize() {
    armWasUp = false;
    climber.setLowerLimit(CLIMB_DESTRUCTION_TICKS);
    desiredTicks = climber.getClimberPosition();
    shooter.setHoodPosition(ShooterConstants.MIN_HOOD_POSITION);
    if (climber.getClimberPosition() >= ClimberConstants.CLIMBER_MIN_TICKS_TO_LOCK) {
      armWasUp = true;
    }
  }

  @Override
  public void execute() {
    if (climber.getClimberPosition() > ClimberConstants.CLIMBER_MIN_TICKS_TO_LOCK
        && climber.getLeftInductiveSensor()
        && climber.getRightInductiveSensor()
        && armWasUp) {
      desiredTicks = ClimberConstants.CLIMBER_LOCKED_TICKS;
    }

    if (moveDown.getAsBoolean()) {
      desiredTicks -= CLIMB_SPEED_ARM_DOWN;
    } else if (moveUp.getAsBoolean()) {
      desiredTicks += CLIMB_SPEED_ARM_UP;
    }

    desiredTicks =
        Math.max(climber.getLowerLimit(), Math.min(climber.getUpperLimit(), desiredTicks));

    climber.setClimberPosition(desiredTicks);
  }
}
