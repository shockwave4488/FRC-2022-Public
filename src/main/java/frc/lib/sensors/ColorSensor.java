package frc.lib.sensors;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensor {

  private final I2C.Port i2cPort;
  private final ColorSensorV3 m_colorSensor;
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final String portName;

  public enum ColorType {
    RED,
    GREEN,
    BLUE,
    YELLOW,
    UNKNOWN
  }

  private static final double CONFIDENCE_THRESHOLD = 0.85;

  // Day values
  static final Color kBlueTarget = new Color(0.17, 0.42, 0.41);
  static final Color kRedTarget = new Color(0.55, 0.33, 0.11);

  /*
  // Night values
  static final Color kBlueTarget = new Color(0.24, 0.48, 0.27);
  static final Color kRedTarget = new Color(0.33, 0.45, 0.22);
  */

  // Constant values
  static final Color kGreenTarget = new Color(0.20, 0.54, 0.25);
  static final Color kYellowTarget = new Color(0.33, 0.53, 0.13);

  public ColorSensor(I2C.Port port) {
    i2cPort = port;
    m_colorSensor = new ColorSensorV3(i2cPort);
    portName = i2cPort.toString();

    m_colorMatcher.setConfidenceThreshold(CONFIDENCE_THRESHOLD);
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
  }

  /**
   * gets red RGB value of color sensor
   *
   * @return red RGB value 0 to 1
   */
  public double getRed() {
    return m_colorSensor.getColor().red;
  }

  /**
   * gets blue RGB value of color sensor
   *
   * @return blue RGB value 0 to 1
   */
  public double getBlue() {
    return m_colorSensor.getColor().blue;
  }

  /**
   * gets green RGB value of color sensor
   *
   * @return green RGB value 0 to 1
   */
  public double getGreen() {
    return m_colorSensor.getColor().green;
  }

  /**
   * gets what color the color sensor sees
   *
   * @return returns enum of color red, blue, yellow, green or unknown
   */
  public ColorType currentColor() {
    ColorType returnColor;
    Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchColor(detectedColor);

    if (match == null) {
      returnColor = ColorType.UNKNOWN;
    } else if (match.color == kBlueTarget) {
      returnColor = ColorType.BLUE;
    } else if (match.color == kRedTarget) {
      returnColor = ColorType.RED;
    } else if (match.color == kGreenTarget) {
      returnColor = ColorType.GREEN;
    } else if (match.color == kYellowTarget) {
      returnColor = ColorType.YELLOW;
    } else {
      returnColor = ColorType.UNKNOWN;
    }

    return returnColor;
  }

  /**
   * gets what color the color sensor sees
   *
   * @return returns string of color red, blue, yellow, green or unknown
   */
  public String colorString() {
    if (currentColor() == ColorType.RED) {
      return "R";
    } else if (currentColor() == ColorType.GREEN) {
      return "G";
    } else if (currentColor() == ColorType.BLUE) {
      return "B";
    } else if (currentColor() == ColorType.YELLOW) {
      return "Y";
    } else {
      return "UNKNOWN";
    }
  }

  public void periodic() {}

  public void updateSmartDashboard() {
    double[] curRGB = {getRed(), getGreen(), getBlue()};
    SmartDashboard.putNumberArray(portName + " raw color: ", curRGB);
    SmartDashboard.putString(portName + " matched color: ", colorString());
  }
}
