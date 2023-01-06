package frc.robot.commands.mock2023.defaults;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.mock2023.Elevator;

public class DefaultElevator extends CommandBase {
  private final Elevator elevator;

  public DefaultElevator(Elevator elevator) {
    this.elevator = elevator;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.setElevatorDown();
  }
}
