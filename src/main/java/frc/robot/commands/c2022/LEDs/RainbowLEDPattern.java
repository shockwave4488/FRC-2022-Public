package frc.robot.commands.c2022.LEDs;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.c2022.LEDController;
import frc.robot.subsystems.c2022.LEDController.Color;

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
