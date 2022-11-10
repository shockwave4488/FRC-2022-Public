package frc.lib.operator;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;

public class DPad extends Button {

  private XboxController controller;
  private Direction direction;
  private int dPadValue;

  public void DPadButton(XboxController controller, Direction direction) {
    this.controller = controller;
    this.direction = direction;
  }

  public static enum Direction {
    UP(0),
    DOWN(180);

    int direction;

    private Direction(int direction) {
      this.direction = direction;
    }
  }

  public boolean get() {
    dPadValue = controller.getPOV();
    return (dPadValue == direction.direction);
  }
}
