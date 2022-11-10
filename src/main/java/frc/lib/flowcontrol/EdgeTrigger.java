package frc.lib.flowcontrol;

/** Used to detect changes in a boolean state */
public class EdgeTrigger {
  private boolean m_feedback;

  /** {@link #EdgeTrigger} with initial value false */
  public EdgeTrigger() {
    this(false);
  }

  /**
   * {@link #EdgeTrigger} with the given initial value
   *
   * @param initial
   */
  public EdgeTrigger(boolean initial) {
    m_feedback = initial;
  }

  /**
   * Manually update the internal state
   *
   * @param trigger Current value of the edge triggered state
   */
  public void update(boolean trigger) {
    m_feedback = trigger;
  }

  /**
   * Returns true if there is a rising edge and updates the internal state
   *
   * @param trigger Current value of edge triggered state
   * @return Rising Edge
   */
  public boolean getRisingUpdate(boolean trigger) {
    boolean toReturn = (trigger && !m_feedback);
    m_feedback = trigger;
    return toReturn;
  }

  /**
   * Returns true if there is a falling edge and updates the internal state
   *
   * @param trigger Current value of edge triggered state
   * @return Falling Edge
   */
  public boolean getFallingUpdate(boolean trigger) {
    boolean toReturn = (!trigger && m_feedback);
    m_feedback = trigger;
    return toReturn;
  }

  /**
   * Returns true if there is a rising edge, does not update the internal state
   *
   * @param trigger Current value of the edge triggered state
   * @return Rising edge
   */
  public boolean getRising(boolean trigger) {
    return (trigger && !m_feedback);
  }

  /**
   * Returns true if there is a falling edge, does not update the internal state
   *
   * @param trigger Current value of the edge triggered state
   * @return Falling edge
   */
  public boolean getFalling(boolean trigger) {
    return (!trigger && m_feedback);
  }
}
