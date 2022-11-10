package frc.lib.operator;

public interface IDeadzoneCalculator {
  /**
   * Deadzone calculation with a given deadzone
   *
   * @param val The value to be deadzoned.
   * @param deadzone The amount that val inputs must be greater than in order for a return > 0.
   * @return The deadzoned value.
   */
  public double deadzone(double val, double deadzone);

  /**
   * @param val The value to be deadzoned.
   * @return The deadzoned value.
   */
  public double deadzone(double val);
}
