package frc.lib.controlsystems;

import java.util.ArrayList;

/**
 * A custom profile for a control system. Can be used for any measurable quantity (speed,
 * displacement, time, etc.)
 */
public class SetPointProfile {
  private ArrayList<Setpoint> m_profile;

  /** Constructs a new, empty {@link #SetPointProfile} */
  public SetPointProfile() {
    m_profile = new ArrayList<Setpoint>();
  }

  private Setpoint nearestLess(double currentPoint) {
    Setpoint toReturn = m_profile.get(0);
    for (int i = 1; i < m_profile.size(); i++) {
      if (m_profile.get(i).point > currentPoint) return toReturn;

      toReturn = m_profile.get(i);
    }
    return toReturn;
  }

  private Setpoint nearestGreater(double currentPoint) {
    Setpoint toReturn = m_profile.get(m_profile.size() - 1);
    for (int i = m_profile.size() - 2; i >= 0; i--) {
      if (m_profile.get(i).point < currentPoint) return toReturn;

      toReturn = m_profile.get(i);
    }
    return toReturn;
  }

  /**
   * Get the value of the path at the current point with a linear interpolation between points above
   * and below current point
   *
   * @param currentPoint point along the profile to measure
   * @return value of the profile at that point
   */
  public double get(double currentPoint) {
    // grab point above
    Setpoint next = nearestGreater(currentPoint);
    // grab point below
    Setpoint prev = nearestLess(currentPoint);
    // calculate slope
    double slope = (next.value - prev.value) / (next.point - prev.point);
    // handle divide by zero in case where points are the same
    slope = Double.isNaN(slope) ? 0 : slope;
    // calculate value at given point based on linear interpolated slope
    // between above and below point
    double pointValue = slope * (currentPoint - prev.point) + prev.value;
    return pointValue;
  }

  /**
   * Adds a point to the path
   *
   * @param point location
   * @param value of point
   */
  public void add(double point, double value) {
    for (int i = 0; i < m_profile.size(); i++) {
      if (m_profile.get(i).point > point) {
        m_profile.add(i, new Setpoint(point, value));
        return;
      }
    }
    // If the point is greatest or the list is empty
    m_profile.add(new Setpoint(point, value));
  }
}
