package frc.robot.commands.eruption.LEDs;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.eruption.LEDController;
import frc.robot.subsystems.eruption.LEDController.Color;

public class RainbowLEDPattern extends CommandBase {
  private final LEDController LEDs;

  public RainbowLEDPattern(LEDController LEDs) {
    this.LEDs = LEDs;
    addRequirements(LEDs);
  }

  @Override
  public void initialize() {
    LEDs.setColor(Color.Rainbow);
  }

  @Override
  public void end(boolean interrupted) {
    LEDs.setColor(Color.Default);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
