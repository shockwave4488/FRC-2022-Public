package frc.robot.subsystems.c2022;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.wpiextensions.ShockwaveSubsystemBase;
import frc.robot.Constants.LEDControllerConstants;

public class LEDController extends ShockwaveSubsystemBase {

  public enum Color {
    Rainbow,
    Default
  }

  private DigitalOutput ledDio0 = new DigitalOutput(LEDControllerConstants.ledDio1);
  private DigitalOutput ledDio1 = new DigitalOutput(LEDControllerConstants.ledDio2);
  private DigitalOutput ledDio2 = new DigitalOutput(LEDControllerConstants.ledDio3);

  private Color currentColor = Color.Default;
  private Color temporaryChangeColor = Color.Default;
  private double temporaryChangeStart;
  private int[] colorPattern = new int[3];

  @Override
  public void periodic() {
    if ((Timer.getFPGATimestamp() * 1000) - temporaryChangeStart > 1500) {
      changeColor(currentColor);
    } else {
      changeColor(temporaryChangeColor);
    }
  }

  /*
   * * Sets the color of the LED Controller.
   *
   * @param color The specified color of the LED Controller List of available
   * colors can be seen in enum Color.
   */
  public void setColor(Color color) {
    currentColor = color;
    changeColor(color);
  }

  private void changeColor(Color color) {
    switch (color) {
      case Rainbow:
        // Rainbow binary pattern
        colorPattern[0] = 1;
        colorPattern[1] = 0;
        colorPattern[2] = 0;
        break;
      case Default:
        // Default binary pattern
        colorPattern[0] = 1;
        colorPattern[1] = 1;
        colorPattern[2] = 1;
        break;
      default:
        // Default binary pattern
        colorPattern[0] = 1;
        colorPattern[1] = 1;
        colorPattern[2] = 1;
        break;
    }
    ledDio0.set(colorPattern[0] == 1);
    ledDio1.set(colorPattern[1] == 1);
    ledDio2.set(colorPattern[2] == 1);
  }

  public void temporaryChangeColor(Color color) {
    temporaryChangeColor = color;
    temporaryChangeStart = Timer.getFPGATimestamp() * 1000;
  }

  /**
   * Returns the current color instance of the LED Controller. Can be used as an additional checker
   * for if statements in routines.
   */
  public Color getColor() {
    return currentColor;
  }

  @Override
  public void updateSmartDashboard() {}

  @Override
  public void zeroSensors() {}

  @Override
  public void setUpTrackables() {}

  @Override
  public void onStart() {}

  @Override
  public void onStop() {}
}
