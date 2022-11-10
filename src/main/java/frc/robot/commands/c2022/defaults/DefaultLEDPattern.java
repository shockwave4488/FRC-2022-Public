package frc.robot.commands.c2022.defaults;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.c2022.LEDController;
import frc.robot.subsystems.c2022.LEDController.Color;

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
