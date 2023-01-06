package frc.robot.commands.c2022.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.c2022.Climber;
import frc.robot.subsystems.c2022.Shooter;
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
    climber.setLowerLimit(CLIMB_DESTRUCTION_TICKS);
    desiredTicks = climber.getClimberPosition();
    shooter.setHoodPosition(ShooterConstants.MIN_HOOD_POSITION);
  }

  @Override
  public void execute() {
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
