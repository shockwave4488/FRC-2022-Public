package frc.lib.wpiextensions;

import edu.wpi.first.wpilibj.Timer;

public class TimerHelper extends Timer {
  public TimerHelper() {
    reset();
  }

  public static int getMatchTimeMillis() {
    return (int) (getFPGATimestamp() * 1000);
  }
}
