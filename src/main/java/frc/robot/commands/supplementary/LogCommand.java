package frc.robot.commands.supplementary;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Names anonymous commands for more informative RunningCommands logs. */
public class LogCommand extends ProxyCommand {
  /**
   * Makes the given command(s) visible to the CommandScheduler with an identity. WARNING: Could
   * interrupt Command Group or running sub-commands with same requirements.
   *
   * @param name Name to be assigned to set of commands
   * @param commands Commands to run sequentially
   */
  public LogCommand(String name, CommandBase... commands) {
    super(
        (commands.length > 1)
            ? new SequentialCommandGroup(commands).withName(name)
            : commands[0].withName(name));
  }
}
