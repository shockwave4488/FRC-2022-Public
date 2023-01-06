package frc.lib.controlsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.function.BiPredicate;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Handles calculating the stability of a device (motor, or otherwise) around a setpoint. Instead of
 * manually setting up and keeping track of cycles in range and configuration, a {@link
 * #DoneCycleMachine} can be constructed to automatically watch the values and ready status
 * periodically.
 *
 * <p>Using this class is simple:
 *
 * <p>1. Create a {@link #DoneCycleMachine} instance in a constructor of a subsystem, command, etc.
 *
 * <p>2. Execute done cycles using {@link #run()} in any periodic function.
 *
 * <p>3. Call the appropriate methods to configure the machine or check stability. Use {@link
 * #changeGoalCondition(BiPredicate)} to change the default comparison between the actual and
 * desired value according to {@link T#equals(Object)}
 *
 * <p>4. Optionally, use {@link edu.wpi.first.wpilibj.smartdashboard.SmartDashboard#putData(String,
 * Sendable) SmartDashboard.putData} to easily show the current status and settings on the
 * dashboard.
 *
 * <p>Note: Operation of the {@link #DoneCycleMachine} is intended to be done on the Command level,
 * or wherever the setpoint is determined. For this to be possible, the {@link #DoneCycleMachine}
 * object should be exposed in some way to the operating class.
 */
public class DoneCycleMachine<T> implements Sendable {
  private boolean shouldRun = true;
  private final Consumer<? super T> controlMethod;
  protected final Supplier<T> actualValue;
  protected T desiredValue;
  protected BiPredicate<T, T> goalCondition; // actual first, desired second
  private int doneCycles = 0;
  private int minDoneCycles;
  private boolean subtractCycles = false;
  private int subtractCycleNum;

  /**
   * Constructs a new {@link #DoneCycleMachine}. See class comment for guidance on how to use.
   *
   * @param actualValue Supplier of the value to compare the desired value to.
   * @param controlMethod Callback invoked when assigning setpoint. A method reference may generate
   *     a warning if a wrapper class of a primitive type needs to be unboxed. When in doubt, resort
   *     to a lambda expression.
   */
  public DoneCycleMachine(Supplier<T> actualValue, Consumer<? super T> controlMethod) {
    this.actualValue = actualValue;
    this.controlMethod = controlMethod;
    goalCondition =
        (actualVal, desiredVal) ->
            (actualVal == null) ? desiredVal == null : actualVal.equals(desiredVal);
  }

  /**
   * Constructs a new {@link #DoneCycleMachine} without a device setter method. See class comment
   * for guidance on how to use.
   *
   * @param actualValue Actual value supplier of the device.
   */
  public DoneCycleMachine(Supplier<T> actualValue) {
    this(actualValue, desiredVal -> {});
  }

  /**
   * {@link #DoneCycleMachine} loop calculating done cycles. Must call periodically for the machine
   * to do its job.
   */
  public void run() {
    if (shouldRun) {
      if (goalCondition.test(actualValue.get(), desiredValue)) {
        doneCycles++;
      } else {
        doneCycles = subtractCycles ? Math.max(doneCycles - subtractCycleNum, 0) : 0;
      }
    }
  }

  /** Enable/disable the {@link #DoneCycleMachine} loop. Will reset done cycle count. */
  public void enable(boolean shouldRun) {
    doneCycles = 0;
    this.shouldRun = shouldRun;
  }

  /**
   * Changes how the actual and desired values are compared to each other to decide when to
   * increment the done cycle count.
   */
  public void changeGoalCondition(BiPredicate<T, T> goalCondition) {
    this.goalCondition = goalCondition;
    resetDoneCycles();
  }

  /** Returns whether the goal condition has been satisfied for the desired amount of time. */
  public boolean isReady() {
    return doneCycles >= minDoneCycles;
  }

  /**
   * Reset done cycle count to ensure {@link #minDoneCycles} more iterations must pass before being
   * ready.
   */
  public void resetDoneCycles() {
    doneCycles = 0;
  }

  /** Set the minimum number of cycles in range/satisfying the goal condition to return ready. */
  public void setMinDoneCycles(int cycles) {
    minDoneCycles = cycles;
  }

  /**
   * Turn on/off subtract-cycles mode. {@link #setSubtractCycles} enables this by itself, so this
   * method is mainly useful for disabling it later.
   */
  public void enableSubtractCycles(boolean turnOn) {
    subtractCycles = turnOn;
  }

  /**
   * Specify the number of cycles by which to reduce the done cycle count after each iteration out
   * of done range or not meeting the goal condition. Automatically enables subtract-cycles mode.
   */
  public void setSubtractCycles(int cycles) {
    enableSubtractCycles(true);
    subtractCycleNum = cycles;
  }

  /**
   * Assign setpoint for the device.
   *
   * @param desiredValue Setpoint
   * @param reset Whether to set the done cycle count to 0, even if current value would meet the new
   *     conditions.
   */
  public void setDesiredValue(T desiredValue, boolean reset) {
    this.desiredValue = desiredValue;
    if (reset) resetDoneCycles();
    controlMethod.accept(desiredValue);
  }

  /**
   * Assign setpoint for the device. Current cycles will be reset.
   *
   * @param desiredValue Setpoint
   */
  public void setDesiredValue(T desiredValue) {
    setDesiredValue(desiredValue, true);
  }

  public T getDesiredValue() {
    return desiredValue;
  }

  public T getActualValue() {
    return actualValue.get();
  }

  public int getDoneCycleCount() {
    return doneCycles;
  }

  protected void addDesiredValueProperty(SendableBuilder builder) {
    builder.addStringProperty(
        "Desired value",
        () -> {
          T desiredVal = getDesiredValue();
          return (desiredVal != null) ? desiredVal.toString() : "null";
        },
        null);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("Enabled", () -> shouldRun, this::enable);
    builder.addStringProperty(
        "Done cycles",
        () ->
            String.valueOf(Math.min(doneCycles, minDoneCycles))
                + " / "
                + String.valueOf(minDoneCycles),
        null);
    addDesiredValueProperty(builder);
    builder.addStringProperty(
        "Actual value",
        () -> {
          T actualVal = getActualValue();
          return (actualVal != null) ? actualVal.toString() : "null";
        },
        null);
  }
}
