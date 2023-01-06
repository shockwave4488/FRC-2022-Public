package frc.robot.commands.mock2023.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.mock2023.Elevator;

public class ElevatorUp extends CommandBase {
  private final Elevator elevator;

  public ElevatorUp(Elevator elevator) {
    this.elevator = elevator;
  }

  /**
   * Use this constructor to set the climber to a certain height without caring about the climber
   * subsystem's isStable method. Since this constructor doesn't care about isStable, this command
   * will instantly end after setting the climber's desired position.
   *
   * @param climber The climber subsystem object
   * @param desiredTicks The desired height of the climber in motor ticks
   */
  @Override
  public void initialize() {
    elevator.setElevatorUp();
  }
}
