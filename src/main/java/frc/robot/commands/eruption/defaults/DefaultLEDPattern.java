package frc.robot.commands.eruption.defaults;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.eruption.LEDController;
import frc.robot.subsystems.eruption.LEDController.Color;

public class DefaultLEDPattern extends CommandBase {
  private final LEDController LEDs;

  public DefaultLEDPattern(LEDController LEDs) {
    this.LEDs = LEDs;
    addRequirements(LEDs);
  }

  public void initialize() {
    LEDs.setColor(Color.Default);
  }
}
