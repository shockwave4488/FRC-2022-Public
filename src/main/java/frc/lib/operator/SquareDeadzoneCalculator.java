package frc.lib.operator;

public class SquareDeadzoneCalculator implements IDeadzoneCalculator {
  private final double DEFAULT_DEADZONE;

  public SquareDeadzoneCalculator(double DEFAULT_DEADZONE) {
    this.DEFAULT_DEADZONE = DEFAULT_DEADZONE;
  }

  public double deadzone(double val) {
    return deadzone(val, DEFAULT_DEADZONE);
  }

  public double deadzone(double val, double deadzone) {
    double deadzonedValue = 0;
    if (Math.abs(val) > Math.abs(deadzone)) {
      if (val > 0) {
        deadzonedValue = (val - deadzone) / (1 - deadzone);
      } else {
        deadzonedValue = (val + deadzone) / (1 - deadzone);
      }
    }

    return deadzonedValue;
  }
}
