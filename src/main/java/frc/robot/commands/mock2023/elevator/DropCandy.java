package frc.robot.commands.mock2023.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.mock2023.Elevator;

public class DropCandy extends CommandBase {

  private final Elevator elevator;

  public DropCandy(Elevator elevator) {
    this.elevator = elevator;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    elevator.openCandyGrabber();
  }
}
