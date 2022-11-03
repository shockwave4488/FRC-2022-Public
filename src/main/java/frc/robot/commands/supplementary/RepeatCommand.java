package frc.robot.commands.supplementary;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;

public class RepeatCommand extends PerpetualCommand {
  private boolean wasFinished;

  /**
   * Restarts the given command when it finishes. This is useful when the behavior of a continuous
   * trigger is desired, but with a single button press or within a command group.
   *
   * @param command Command to repeat indefinitely
   */
  public RepeatCommand(Command command) {
    super(command);
    this.setName("Repeat" + command.getName());
  }

  @Override
  public void initialize() {
    super.initialize();
    wasFinished = false;
  }

  @Override
  public void execute() {
    if (wasFinished) {
      m_command.initialize();
      wasFinished = false;
    }
    m_command.execute();
    if (m_command.isFinished()) {
      m_command.end(false);
      wasFinished = true;
    }
  }
}
