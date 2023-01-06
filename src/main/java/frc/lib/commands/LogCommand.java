package frc.lib.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.supplementary.RepeatCommand;
import java.util.Collection;
import java.util.function.BooleanSupplier;
import java.util.stream.Stream;

/**
 * Uncovers hidden commands for more informative RunningCommands logs.
 *
 * <p>Suppose there is a top-level command group containing a few commands. By default, none of
 * those sub-commands, as they execute, are visible to the CommandScheduler and therefore logging.
 * Only a generic name will show, which is typically insufficient for debugging the actual sequence
 * of events as they took place. Luckily, LogCommands together with the {@link CommandLogger} solve
 * this issue.
 *
 * <p>In most cases, usage of this class should be limited to the static factory methods {@link
 * #sequence(Command...) sequence()}, {@link #parallel(Command...) parallel()} {@link
 * #race(Command...) race()}, {@link #deadline(Command, Command...) deadline()}, {@link
 * #conditional(Command, Command, BooleanSupplier) conditional()}, etc. for making command
 * groups/wrappers with logged sub-commands. The main exception is when calling {@link
 * CommandGroupBase#addCommands(Command...) addCommands()} when extending a CommandGroup. In that
 * situation, the inside LogCommand function call should be {@link #arrayOf(Command, Command...)
 * arrayOf(this, [sub-commands])} in order to properly nest the commands in the outer group.
 *
 * <p>For brevity purposes, it is recommended to static import the methods of this class when
 * nesting many command groups.
 */
public class LogCommand extends CommandBase {
  private static final CommandLogger commandLogger = CommandLogger.getInstance();
  private final Command command;
  private Command parent = null;

  /**
   * Exposes the given command to the command logging mechanism. Usually there is no need to use
   * this constructor directly.
   *
   * <p>Note that LogCommands do not result in Grouped warnings! Do <b>not</b> independently
   * schedule or add {@code command} to a command group once it has been wrapped in a LogCommand.
   * Doing so anyway could put it in an inconsistent internal state. Instead, create another command
   * instance.
   *
   * @param command Command to log.
   * @see {@link CommandLogger}
   */
  public LogCommand(Command command) {
    this.command = command;
    m_requirements.addAll(command.getRequirements());
    setName(command.getName());
    commandLogger.registerCommand(command, this);
  }

  public static LogCommand[] arrayOf(Command parent, Command... commands) {
    return Stream.of(commands)
        .map(cmd -> new LogCommand(cmd).withParent(parent))
        .toArray(LogCommand[]::new);
  }

  public static <T extends CommandGroupBase> T addTo(T commandGroup, Command... commands) {
    commandGroup.addCommands(arrayOf(commandGroup, commands));
    return commandGroup;
  }

  /** Factory method to create a {@link SequentialCommandGroup} with logged {@code commands}. */
  public static SequentialCommandGroup sequence(Command... commands) {
    return (SequentialCommandGroup)
        addTo(new SequentialCommandGroup(), commands).withName("Sequence");
  }

  /** Factory method to create a {@link ParallelCommandGroup} with logged {@code commands}. */
  public static ParallelCommandGroup parallel(Command... commands) {
    return (ParallelCommandGroup) addTo(new ParallelCommandGroup(), commands).withName("Parallel");
  }

  /** Factory method to create a {@link ParallelRaceGroup} with logged {@code commands}. */
  public static ParallelRaceGroup race(Command... commands) {
    return (ParallelRaceGroup) addTo(new ParallelRaceGroup(), commands).withName("Race");
  }

  /**
   * Factory method to create a {@link ParallelDeadlineGroup} with logged {@code deadline} and
   * {@code commands}.
   */
  public static ParallelDeadlineGroup deadline(Command deadline, Command... commands) {
    LogCommand deadlineCommand = new LogCommand(deadline);
    ParallelDeadlineGroup deadlineGroup = new ParallelDeadlineGroup(deadlineCommand);
    deadlineGroup.setName("Deadline");
    deadlineCommand.setParent(deadlineGroup);
    return addTo(deadlineGroup, commands);
  }

  /**
   * Factory method to create a {@link ConditionalCommand} with logged {@code onTrue} and {@code
   * onFalse}.
   */
  public static ConditionalCommand conditional(
      Command onTrue, Command onFalse, BooleanSupplier condition) {
    LogCommand onTrueCommand = new LogCommand(onTrue);
    LogCommand onFalseCommand = new LogCommand(onFalse);
    ConditionalCommand conditionalCommand =
        new ConditionalCommand(onTrueCommand, onFalseCommand, condition);
    conditionalCommand.setName("Conditional");
    onTrueCommand.setParent(conditionalCommand);
    onFalseCommand.setParent(conditionalCommand);
    return conditionalCommand;
  }

  /** Factory method to create a {@link ProxyScheduleCommand} with logged {@code commands}. */
  public static ProxyScheduleCommand proxy(Command... commands) {
    LogCommand[] proxyLogCommands = arrayOf(null, commands);
    ProxyScheduleCommand proxyScheduleCommand = new ProxyScheduleCommand(proxyLogCommands);
    proxyScheduleCommand.setName("ProxySchedule");
    for (LogCommand command : proxyLogCommands) {
      command.setParent(proxyScheduleCommand);
    }
    return proxyScheduleCommand;
  }

  /** Factory method to create a {@link RepeatCommand} with logged {@code command}. */
  public static RepeatCommand repeat(Command command) {
    LogCommand repeatLogCommand = new LogCommand(command);
    RepeatCommand repeatCommand = new RepeatCommand(repeatLogCommand);
    repeatCommand.setName("Repeat");
    repeatLogCommand.setParent(repeatCommand);
    return repeatCommand;
  }

  /** Factory method to create a {@link PerpetualCommand} with logged {@code command}. */
  public static PerpetualCommand perpetual(Command command) {
    LogCommand perpetualLogCommand = new LogCommand(command);
    PerpetualCommand perpetualCommand = new PerpetualCommand(perpetualLogCommand);
    perpetualCommand.setName("Perpetual");
    perpetualLogCommand.setParent(perpetualCommand);
    return perpetualCommand;
  }

  /**
   * {@link Command#withInterrupt(BooleanSupplier)} / {@link Command#until(BooleanSupplier)} that
   * preserves the original command name.
   */
  public static ParallelRaceGroup endWhen(Command command, BooleanSupplier condition) {
    return (ParallelRaceGroup) command.withInterrupt(condition).withName(command.getName());
  }

  /** {@link Command#withTimeout(double)} that preserves the original command name. */
  public static ParallelRaceGroup endAfter(Command command, double seconds) {
    return (ParallelRaceGroup) command.withTimeout(seconds).withName(command.getName());
  }

  @Override
  public LogCommand withName(String name) {
    return (LogCommand) super.withName(name);
  }

  String getLogName() {
    return (hasParent() ? commandLogger.getLogName(parent) + " -> " : "") + getName();
  }

  public void setParent(Command parent) {
    this.parent = parent;
  }

  public LogCommand withParent(Command parent) {
    setParent(parent);
    return this;
  }

  public boolean hasParent() {
    return parent != null;
  }

  boolean parentIn(Collection<? extends Command> collection) {
    if (hasParent()) {
      return collection.stream()
          .anyMatch(
              cmd -> {
                int commandHashCode =
                    (cmd instanceof LogCommand)
                        ? ((LogCommand) cmd).getCommandHashCode()
                        : cmd.hashCode();
                return commandHashCode == getParentHashCode();
              });
    }
    return false;
  }

  int getParentHashCode() {
    return hasParent() ? parent.hashCode() : -1;
  }

  int getCommandHashCode() {
    return command.hashCode();
  }

  @Override
  public void initialize() {
    command.initialize();
  }

  @Override
  public void execute() {
    command.execute();
    commandLogger.logCommand(this);
  }

  @Override
  public void end(boolean interrupted) {
    command.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return command.isFinished();
  }

  @Override
  public boolean runsWhenDisabled() {
    return command.runsWhenDisabled();
  }
}
