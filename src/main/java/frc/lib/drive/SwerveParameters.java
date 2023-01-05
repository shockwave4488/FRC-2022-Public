package frc.lib.drive;

public class SwerveParameters {
  public int driveMotorChannel;
  public int turningMotorChannel;
  public int turningEncoderChannel;
  public double potOffset;
  public int potResolution;
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
   * @param potOffset Potentiometer/angle offset value. Should be in ticks for neos and radians for
   *     falcons.
   * @param potResolution Resolution of the potentiometer.
   * @param wheelDiameter Diameter of the wheel in meters.
   * @param driveGearRatio Gear ratio of the motor to wheel (how many times the motor spins for the
   *     wheel to spin once)
   * @param modulePosition Position of the module.
   */
  public SwerveParameters(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      double potOffset,
      int potResolution,
      double wheelDiameter,
      double driveGearRatio,
      ModulePosition modulePosition) {
    this.driveMotorChannel = driveMotorChannel;
    this.turningMotorChannel = turningMotorChannel;
    this.turningEncoderChannel = turningEncoderChannel;
    this.potOffset = potOffset;
    this.potResolution = potResolution;
    this.wheelDiameter = wheelDiameter;
    this.driveGearRatio = driveGearRatio;
    this.modulePosition = modulePosition;
  }

  public SwerveParameters() {
    // Called via Util.toObj()
  }
}
