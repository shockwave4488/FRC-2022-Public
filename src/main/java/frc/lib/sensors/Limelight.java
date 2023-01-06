package frc.lib.sensors;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.flowcontrol.EdgeTrigger;
import frc.lib.logging.Logger;
import frc.lib.wpiextensions.ShockwaveSubsystemBase;
import frc.robot.Constants.UnitConversionConstants;
import java.util.Optional;

public class Limelight extends ShockwaveSubsystemBase {

  private final String name;
  private final Logger logger;

  private static final int DEFAULT_PIPE = 0;

  private NetworkTable table;
  private NetworkTableEntry xEntry, yEntry, areaEntry;
  private NetworkTableEntry hasTargetEntry, currentPipeEntry;
  private NetworkTableEntry ledControlEntry, pipeControlEntry;

  private Optional<DigitalOutput> extraLedDio;

  private SendableChooser<Boolean> forceLedSelector = new SendableChooser<Boolean>();
  private EdgeTrigger selectedLedOn = new EdgeTrigger(false);

  public enum LedControl {
    PipeControl(0),
    ForceOff(1),
    ForceOn(3),
    ForceBlink(2);

    public final int val;

    private LedControl(int val) {
      this.val = val;
    }
  }

  public static class DistanceEstimationConstants {
    public double camHeight;
    public double targetHeight;
    public double camToNormalAngle;

    public DistanceEstimationConstants(
        double camHeight, double targetHeight, double camToNormalAngle) {
      this.camHeight = camHeight;
      this.targetHeight = targetHeight;
      this.camToNormalAngle = camToNormalAngle;
    }
  }

  private final DistanceEstimationConstants distEstConsts;

  public Limelight(String name, Logger logger) {
    this(name, new DistanceEstimationConstants(0, 0, 0), logger);
  }

  /*
  Note that passing in a SetPointProfile (interpolation table) through this constructor has been done in the past to have an internal calculation of distance from a vision target.
  */
  public Limelight(String name, DistanceEstimationConstants distEstConsts, Logger logger) {
    this.name = name;
    this.distEstConsts = distEstConsts;
    this.logger = logger;
    extraLedDio = Optional.empty();
    updateTables();
    setLed(LedControl.ForceOff);
    setPipeline(DEFAULT_PIPE);

    forceLedSelector.setDefaultOption(name + "-NormalLedControl", false);
    forceLedSelector.addOption(name + "-ForceLedOn", true);
    SmartDashboard.putData(forceLedSelector);
  }

  public Limelight(
      String name, DistanceEstimationConstants distEstConsts, int secondLedDio, Logger logger) {
    this(name, distEstConsts, logger);
    extraLedDio = Optional.of(new DigitalOutput(secondLedDio));
  }

  public void updateTables() {
    table = NetworkTableInstance.getDefault().getTable(name);
    xEntry = table.getEntry("tx");
    yEntry = table.getEntry("ty");
    areaEntry = table.getEntry("ta");
    hasTargetEntry = table.getEntry("tv");
    currentPipeEntry = table.getEntry("getpipe");
    ledControlEntry = table.getEntry("ledMode");
    pipeControlEntry = table.getEntry("pipeline");
  }

  public boolean hasTarget() {
    return hasTargetEntry.getDouble(0.0) == 1.0;
  }

  public int getRunningPipeline() {
    return (int) currentPipeEntry.getDouble(0.0);
  }

  /**
   * Gets the horizontal angle difference (in degrees) between the center of the limelight's view
   * and where its detected target is on its view.
   *
   * @return Horizontal angle difference in degrees
   */
  public double getX() {
    return xEntry.getDouble(0.0);
  }

  /**
   * Gets the vertical angle difference (in degrees) between the center of the limelight's view and
   * where its detected target is on its view.
   *
   * @return Vertical angle difference in degrees
   */
  public double getY() {
    return yEntry.getDouble(0.0);
  }

  public double getArea() {
    return areaEntry.getDouble(0.0);
  }

  /**
   * Estimates the translational distance (not including height) from the limelight to the
   * reflective tape
   *
   * @return Translational distance to reflective tape in meters
   */
  public double getEstimatedDistance() {
    double distInInches =
        (distEstConsts.targetHeight - distEstConsts.camHeight)
            / Math.tan((distEstConsts.camToNormalAngle + getY()) * Math.PI / 180);
    return distInInches * UnitConversionConstants.INCHES_TO_METERS;
  }

  public void setLed(LedControl controlMode) {
    ledControlEntry.setNumber(controlMode.val);
    extraLedDio.ifPresent(out -> out.set(controlMode == LedControl.ForceOn));
  }

  public void setPipeline(int pipeline) {
    pipeControlEntry.setNumber(pipeline);
  }

  @Override
  public void periodic() {
    updateTables();
  }

  /**
   * @param currentAngle The current angle of the robot, in degrees
   * @param defaultAngle The rotation to turn to if the limelight doesn't have the target, in
   *     degrees
   * @return If the limelight can see a target, the angle needed to face the center of that target,
   *     otherwise, this returns a default angle.
   */
  public Rotation2d getDesiredAngle(double currentAngle, double defaultAngle) {
    double targetAngle = defaultAngle;
    if (hasTarget()) {
      double limelightAngle = getX();
      // Returns a negative value when the target is on the left side of the screen
      // SmartDashboard.putNumber("LL X Difference", limelightAngle);
      SmartDashboard.putNumber("LL Robot Current Angle", currentAngle);
      targetAngle = currentAngle - limelightAngle;
      if (targetAngle > 180) {
        targetAngle -= 360;
      } else if (targetAngle < -180) {
        targetAngle += 360;
      }
    }

    targetAngle *= Math.PI / 180; // convert to radians
    return new Rotation2d(targetAngle);
  }

  public void updateSmartDashboard() {
    SmartDashboard.putNumber(name + "-X", getX());
    SmartDashboard.putNumber(name + "-Y", getY());
    SmartDashboard.putBoolean(name + "-HasTarget", hasTarget());
    SmartDashboard.putNumber(name + "-Area", getArea());
    SmartDashboard.putNumber(name + "-EstimatedDistance", getEstimatedDistance());
    // SmartDashboard.putNumber(name + "-Pipe", getRunningPipeline());

    if (forceLedSelector.getSelected().booleanValue()) {
      setLed(LedControl.ForceOn);
      selectedLedOn.update(true);
    } else if (selectedLedOn.getFallingUpdate(forceLedSelector.getSelected().booleanValue()))
      setLed(LedControl.ForceOff);
  }

  @Override
  public void onStart() {
    updateTables();
    setLed(LedControl.ForceOn);
  }

  @Override
  public void onStop() {
    setLed(LedControl.ForceOff);
  }

  @Override
  public void zeroSensors() {}

  @Override
  public void setUpTrackables() {
    logger.addTrackable(this::getEstimatedDistance, name + "_EstimatedDistance", 10);
    logger.addTrackable(this::getX, name + "_X", 10);
    logger.addTrackable(this::getY, name + "_Y", 10);
    logger.addTrackable(() -> hasTarget() ? 1 : 0, name + "_HasTarget", 10);
  }
}
