package frc.lib.controlsystems;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.lib.util.app.Util;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Extension of {@link DoneCycleMachine} for numbers which offers a tolerance for error via {@link
 * #setDoneRange(double)} as is required for most applications.
 */
public class NumberDoneCycleMachine extends DoneCycleMachine<Double> {
  private double doneRange;

  /**
   * Docs at {@link frc.lib.controlsystems.DoneCycleMachine#DoneCycleMachine(Supplier, Consumer)
   * DoneCycleMachine}.
   */
  public NumberDoneCycleMachine(
      Supplier<Double> actualValue, Consumer<? super Double> controlMethod) {
    super(actualValue, controlMethod);
    changeGoalCondition(
        (actualVal, desiredVal) -> {
          requireNonNullParam(actualVal, "actualVal", "DoneCycleMachine condition");
          requireNonNullParam(desiredVal, "desiredVal", "DoneCycleMachine condition");
          return Util.epsilonEquals(actualVal, desiredVal, doneRange);
        });
    desiredValue = 0.0;
  }

  /**
   * Docs at {@link frc.lib.controlsystems.DoneCycleMachine#DoneCycleMachine(Supplier)
   * DoneCycleMachine}.
   */
  public NumberDoneCycleMachine(Supplier<Double> actualValue) {
    this(actualValue, desiredVal -> {});
  }

  /**
   * Set the tolerance for the actual value to be accepted even if it isn't precisely at the desired
   * value.
   *
   * @param resetCycles Whether to set the done cycle count to 0, even if current value would be
   *     within the new range
   */
  public void setDoneRange(double errorEpsilon, boolean resetCycles) {
    doneRange = errorEpsilon;
    if (resetCycles) resetDoneCycles();
  }

  /**
   * Set the tolerance for the actual value to be accepted even if it isn't precisely at the desired
   * value. Resets done cycle count.
   */
  public void setDoneRange(double errorEpsilon) {
    setDoneRange(errorEpsilon, true);
  }

  @Override
  protected void addDesiredValueProperty(SendableBuilder builder) {
    builder.addDoubleProperty(
        "Desired value",
        () -> {
          Double desiredVal = getDesiredValue();
          return (desiredVal != null) ? desiredVal.doubleValue() : -1;
        },
        this::setDesiredValue);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty(
        "Done range", () -> doneRange, errorEpsilon -> doneRange = errorEpsilon);
  }
}
