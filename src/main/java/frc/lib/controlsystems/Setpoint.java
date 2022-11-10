package frc.lib.controlsystems;

/** Describes a set point for a profiling system. */
public class Setpoint {
  public Setpoint() {}

  /** Location or set point */
  public double point;
  /** Value at the set point */
  public double value;

  /**
   * new SetPoint
   *
   * @param point
   * @param value
   */
  public Setpoint(double point, double value) {
    this.point = point;
    this.value = value;
  }
}
