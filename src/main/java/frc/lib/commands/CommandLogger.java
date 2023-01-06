package frc.lib.commands;

import edu.umd.cs.findbugs.annotations.SuppressFBWarnings;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.logging.Logger;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

public final class CommandLogger implements NTSendable, AutoCloseable {
  private static CommandLogger instance;

  /** Returns the CommandLogger singleton. */
  @SuppressFBWarnings("MS_EXPOSE_REP")
  public static synchronized CommandLogger getInstance() {
    if (instance == null) {
      instance = new CommandLogger();
    }
    return instance;
  }

  private boolean m_disabled;
  private final Map<Command, LogCommand> m_loggedCommands = new HashMap<>();
  private final Set<LogCommand> m_runningCommands = new HashSet<>();
  private String[] loggedNames = new String[0];

  private CommandLogger() {
    SendableRegistry.addLW(this, "CommandLogger");

    CommandScheduler.getInstance()
        .onCommandExecute(
            command -> {
              boolean isLogCommand = command instanceof LogCommand;
              if (!m_loggedCommands.containsKey(command) && !isLogCommand) {
                registerCommand(command, new LogCommand(command));
              }
              logCommand(isLogCommand ? (LogCommand) command : m_loggedCommands.get(command));
            });
  }

  public void setLoggerTrackable(Logger logger) {
    logger.addStringTrackable(
        () -> (String.join("  |  ", loggedNames)),
        "CommandLogger",
        4,
        "Running Commands:  Command group -> Sub-command");
  }

  void registerCommand(Command baseCommand, LogCommand logCommand) {
    m_loggedCommands.put(baseCommand, logCommand);
  }

  /**
   * Adds a {@link LogCommand} to the current set of running commands. Since the running commands
   * are periodically refreshed, this method must be called every cycle while the command is
   * running.
   */
  void logCommand(LogCommand command) {
    if (!m_disabled) {
      m_runningCommands.add(command);
    }
  }

  String getLogName(Command command) {
    LogCommand logCommand = m_loggedCommands.get(command);
    return (logCommand != null) ? logCommand.getLogName() : command.getName();
  }

  @Override
  public void close() {
    SendableRegistry.remove(this);
  }

  /**
   * Constructs a list of names of the logged commands and clears the previous iteration of running
   * commands. Intended to be called after the CommandScheduler runs in {@link
   * frc.robot.Robot#robotPeriodic() robotPeriodic()}.
   */
  public void update() {
    if (!m_disabled) {
      Map<Double, String> runningCommandNames = new HashMap<>();
      Set<Double> runningParents = new HashSet<>();

      for (LogCommand command : m_runningCommands) {
        boolean parentLogged = command.parentIn(m_runningCommands);
        if (parentLogged) {
          runningParents.add((double) command.getParentHashCode());
        }
        runningCommandNames.put((double) command.getCommandHashCode(), command.getLogName());
      }

      // If a sub-command is running (and printed), don't log that the command group is running.
      runningParents.forEach(parentId -> runningCommandNames.remove(parentId));
      loggedNames = runningCommandNames.values().toArray(new String[0]);

      m_runningCommands.clear();
    }
  }

  /** Disables the command logger. */
  public void disable() {
    m_disabled = true;
  }

  /** Enables the command logger. */
  public void enable() {
    m_disabled = false;
  }

  @Override
  public void initSendable(NTSendableBuilder builder) {
    builder.setSmartDashboardType("CommandLogger");
    builder.addStringArrayProperty("Names", () -> loggedNames, null);
  }
}
