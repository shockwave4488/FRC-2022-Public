package frc.lib.controlsystems;

import static org.junit.Assert.assertEquals;

import org.junit.Before;
import org.junit.Test;

public class SimPIDTest {
  SimPID pid;
  static final double epsilon = 0.01;
  static final double P = 0.15;
  static final double I = 0.0;
  static final double D = 0.0;

  @Before
  public void setup() {
    pid = new SimPID(P, I, D);
    pid.setDoneRange(1);
    pid.setErrorEpsilon(0);
  }

  @Test
  public void test() {
    // Set target for PID
    final int stepSize = 1;
    final int target = 5;
    pid.setDesiredValue(target);

    // Check value of PID with P alone
    for (int i = 0; i < 6; i += stepSize) {
      // Expected power = P * error
      assertEquals((5 - i) * 0.15, pid.calcPID(i), epsilon);
    }
  }
}
