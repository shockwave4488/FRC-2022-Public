package frc.lib.drive;

import frc.lib.PreferenceDoesNotExistException;
import java.util.ArrayList;
import org.json.simple.JSONObject;

public class SwerveParameters {
  public int driveMotorChannel;
  public int turningMotorChannel;
  public int turningEncoderChannel;
  public double absoluteEncoderOffset;
  public int absoluteEncoderResolution;
  public int relativeTurningEncoderResolution;
  public int driveEncoderResolution;
  public double wheelDiameter;
  public double driveGearRatio;
  public ModulePosition modulePosition;

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
   * @param absoluteEncoderOffset Angle encoder offset value. Should be in ticks for neos and
   *     radians for falcons.
   * @param absoluteEncoderResolution Resolution of the absolute angle encoder used to determine the
   *     {@code absoluteEncoderOffset}.
   * @param relativeTurningEncoderResolution Resolution of the potentiometer/encoder used for
   *     turning PID control
   * @param driveEncoderResolution Resolution of the (likely integrated) encoder used for velocity
   *     control of the drive motor.
   * @param wheelDiameter Diameter of the wheel in meters.
   * @param driveGearRatio Gear ratio of the motor to wheel (how many times the motor spins for the
   *     wheel to spin once)
   * @param modulePosition Position of the module.
   */
  public SwerveParameters(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      double absoluteEncoderOffset,
      int absoluteEncoderResolution,
      int relativeTurningEncoderResolution,
      int driveEncoderResolution,
      double wheelDiameter,
      double driveGearRatio,
      ModulePosition modulePosition) {
    this.driveMotorChannel = driveMotorChannel;
    this.turningMotorChannel = turningMotorChannel;
    this.turningEncoderChannel = turningEncoderChannel;
    this.absoluteEncoderOffset = absoluteEncoderOffset;
    this.absoluteEncoderResolution = absoluteEncoderResolution;
    this.relativeTurningEncoderResolution = relativeTurningEncoderResolution;
    this.driveEncoderResolution = driveEncoderResolution;
    this.wheelDiameter = wheelDiameter;
    this.driveGearRatio = driveGearRatio;
    this.modulePosition = modulePosition;
  }

  public SwerveParameters(JSONObject jsonObject) {
    ArrayList<String> keyList = new ArrayList<String>();
    keyList.add(0, "driveMotorChannel");
    keyList.add(1, "turningMotorChannel");
    keyList.add(2, "turningEncoderChannel");
    keyList.add(3, "absoluteEncoderOffset");
    keyList.add(4, "absoluteEncoderResolution");
    keyList.add(5, "relativeTurningEncoderResolution");
    keyList.add(6, "driveEncoderResolution");
    keyList.add(7, "wheelDiameter");
    keyList.add(8, "driveGearRatio");
    keyList.add(9, "modPosition");

    for (int i = 0; i < keyList.size(); i++) {
      if (jsonObject.get(keyList.get(i)) == null) {
        throw new PreferenceDoesNotExistException(
            keyList.get(i) + " in " + this.getClass().getSimpleName());
      }
    }

    this.driveMotorChannel = ((Long) jsonObject.get(keyList.get(0))).intValue();
    this.turningMotorChannel = ((Long) jsonObject.get(keyList.get(1))).intValue();
    this.turningEncoderChannel = ((Long) jsonObject.get(keyList.get(2))).intValue();
    this.absoluteEncoderOffset = ((Number) jsonObject.get(keyList.get(3))).doubleValue();
    this.absoluteEncoderResolution = ((Long) jsonObject.get(keyList.get(4))).intValue();
    this.relativeTurningEncoderResolution = ((Long) jsonObject.get(keyList.get(5))).intValue();
    this.driveEncoderResolution = ((Long) jsonObject.get(keyList.get(6))).intValue();
    this.wheelDiameter = ((Number) jsonObject.get(keyList.get(7))).doubleValue();
    this.driveGearRatio = ((Number) jsonObject.get(keyList.get(8))).doubleValue();
    this.modulePosition = ModulePosition.valueOf(jsonObject.get(keyList.get(9)).toString());
  }
}
