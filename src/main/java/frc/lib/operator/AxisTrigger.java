package frc.lib.operator;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AxisTrigger extends Trigger {

  private final XboxController controller;
  private final int port;
  private final double threshold;

  public AxisTrigger(XboxController controller, int port, double threshold) {
    this.controller = controller;
    this.port = port;
    this.threshold = Math.min(Math.max(threshold, 0), 1);
  }

  @Override
  public boolean get() {
    return controller.getRawAxis(port) >= threshold;
  }
}
