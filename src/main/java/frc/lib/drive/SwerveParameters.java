package frc.lib.drive;

import frc.lib.PreferenceDoesNotExistException;
import java.util.ArrayList;
import org.json.simple.JSONObject;

public class SwerveParameters {
  public int driveMotorChannel;
  public int turningMotorChannel;
  public int turningEncoderChannel;
  public double potOffset;
  public int potResolution;
  public double wheelDiameter;
  public double driveGearRatio;
  public ModulePosition modulePosition;
  public double moduleX;
  public double moduleY;

  public enum ModulePosition {
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT;
  }

  /**
   * Information that will be passed to the SwerveModule
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   * @param turningEncoderChannel ID for the turning encoder.
   * @param potOffset Potentiometer/angle offset value. Should be in ticks for neos and radians for
   *     falcons.
   * @param potResolution Resolution of the potentiometer.
   * @param wheelDiameter Diameter of the wheel in meters.
   * @param driveGearRatio Gear ratio of the motor to wheel (how many times the motor spins for the
   *     wheel to spin once)
   * @param modulePosition Textual position of the module.
   * @param moduleX X coordinate of the module.
   * @param moduleY Y coordinate of the module.
   */
  public SwerveParameters(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      double potOffset,
      int potResolution,
      double wheelDiameter,
      double driveGearRatio,
      ModulePosition modulePosition,
      double moduleX,
      double moduleY) {
    this.driveMotorChannel = driveMotorChannel;
    this.turningMotorChannel = turningMotorChannel;
    this.turningEncoderChannel = turningEncoderChannel;
    this.potOffset = potOffset;
    this.potResolution = potResolution;
    this.wheelDiameter = wheelDiameter;
    this.driveGearRatio = driveGearRatio;
    this.modulePosition = modulePosition;
    this.moduleX = moduleX;
    this.moduleY = moduleY;
  }

  public SwerveParameters(JSONObject jsonObject) {
    ArrayList<String> keyList = new ArrayList<String>();
    keyList.add(0, "driveMotorChannel");
    keyList.add(1, "turningMotorChannel");
    keyList.add(2, "turningEncoderChannel");
    keyList.add(3, "potOffset");
    keyList.add(4, "potResolution");
    keyList.add(5, "wheelDiameter");
    keyList.add(6, "driveGearRatio");
    keyList.add(7, "modPosition");
    keyList.add(8, "modX");
    keyList.add(9, "modY");

    for (int i = 0; i < keyList.size(); i++) {
      if (jsonObject.get(keyList.get(i)) == null) {
        throw new PreferenceDoesNotExistException(
            keyList.get(i) + " in " + this.getClass().getSimpleName());
      }
    }

    this.driveMotorChannel = ((Long) jsonObject.get(keyList.get(0))).intValue();
    this.turningMotorChannel = ((Long) jsonObject.get(keyList.get(1))).intValue();
    this.turningEncoderChannel = ((Long) jsonObject.get(keyList.get(2))).intValue();
    this.potOffset = ((Number) jsonObject.get(keyList.get(3))).doubleValue();
    this.potResolution = ((Long) jsonObject.get(keyList.get(4))).intValue();
    this.wheelDiameter = ((Number) jsonObject.get(keyList.get(5))).doubleValue();
    this.driveGearRatio = ((Number) jsonObject.get(keyList.get(6))).doubleValue();
    this.modulePosition = ModulePosition.valueOf(jsonObject.get(keyList.get(7)).toString());
    this.moduleX = ((Number) jsonObject.get(keyList.get(8))).doubleValue();
    this.moduleY = ((Number) jsonObject.get(keyList.get(9))).doubleValue();
  }
}
