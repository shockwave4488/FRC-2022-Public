package frc.lib.operator;

public class CircularDeadzone implements I2DDeadzoneCalculator {
  private final double DEFAULT_DEADZONE;

  public CircularDeadzone(double DEFAULT_DEADZONE) {
    this.DEFAULT_DEADZONE = DEFAULT_DEADZONE;
  }

  public double[] deadzone(double xVal, double yVal) {
    return deadzone(xVal, yVal, DEFAULT_DEADZONE);
  }

  public double[] deadzone(double xVal, double yVal, double deadzone) {
    double distance = Math.sqrt(Math.pow(xVal, 2) + Math.pow(yVal, 2));
    boolean negativeX = xVal < 0;
    boolean negativeY = yVal < 0;
    if (distance < deadzone) {
      xVal = 0;
      yVal = 0;
    } else {
      distance = (distance - deadzone) / (1 - deadzone);
      double theta = Math.abs(Math.atan(yVal / xVal));
      yVal = Math.sin(theta) * distance;
      xVal = Math.cos(theta) * distance;
      xVal = negativeX ? xVal * -1 : xVal;
      yVal = negativeY ? yVal * -1 : yVal;
    }
    return new double[] {xVal, yVal};
  }

  public boolean isPastDeadzone(double xVal, double yVal) {
    double distance = Math.sqrt(Math.pow(xVal, 2) + Math.pow(yVal, 2));
    return (distance > DEFAULT_DEADZONE);
  }
}
