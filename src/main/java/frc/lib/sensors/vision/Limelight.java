package frc.lib.sensors.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.logging.Logger;
import frc.lib.sensors.vision.VisionTargets.LimelightTarget;
import frc.robot.Constants.FieldConstants;
import java.util.List;
import java.util.Optional;
import org.photonvision.common.hardware.VisionLEDMode;

public class Limelight extends VisionCamera<LimelightTarget> {
  private final String name;
  private final Logger logger;

  private static final int DEFAULT_PIPE = 0;
  private int snapshotCycles = 1;
  private LimelightTarget curTarget;

  private NetworkTableEntry xEntry, yEntry, areaEntry, skewEntry;
  private NetworkTableEntry hasTargetEntry, currentPipeEntry;
  private NetworkTableEntry ledControlEntry, pipeControlEntry;
  private NetworkTableEntry snapshotEntry;

  private Optional<DigitalOutput> extraLedDio;

  public Limelight(String name, Logger logger) {
    this(name, new CameraPositionConstants(new Transform3d()), logger);
  }

  public Limelight(String name, CameraPositionConstants cameraConsts, Logger logger) {
    super(name, cameraConsts, logger);
    this.name = name;
    this.logger = logger;
    extraLedDio = Optional.empty();
    setupTables();
    setLed(VisionLEDMode.kOff);
    setPipeline(DEFAULT_PIPE);
  }

  public Limelight(
      String name, CameraPositionConstants cameraConsts, int secondLedDio, Logger logger) {
    this(name, cameraConsts, logger);
    extraLedDio = Optional.of(new DigitalOutput(secondLedDio));
  }

  private void setupTables() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
    xEntry = table.getEntry("tx");
    yEntry = table.getEntry("ty");
    areaEntry = table.getEntry("ta");
    skewEntry = table.getEntry("ts");
    hasTargetEntry = table.getEntry("tv");
    currentPipeEntry = table.getEntry("getpipe");
    ledControlEntry = table.getEntry("ledMode");
    pipeControlEntry = table.getEntry("pipeline");
    snapshotEntry = table.getEntry("snapshot");
  }

  @Override
  public boolean hasTargets() {
    return hasTargetEntry.getDouble(0.0) == 1.0;
  }

  @Override
  public int getRunningPipeline() {
    return (int) currentPipeEntry.getDouble(0.0);
  }

  @Override
  public Optional<LimelightTarget> getBestTarget() {
    return Optional.ofNullable(curTarget);
  }

  @Override
  public List<LimelightTarget> getTargets() {
    return getBestTarget().map(List::of).orElse(List.of());
  }

  @Override
  public void setLed(VisionLEDMode controlMode) {
    int ledEntryValue;
    switch (controlMode) {
      case kOff:
        ledEntryValue = 1;
        break;
      case kBlink:
        ledEntryValue = 2;
        break;
      case kOn:
        ledEntryValue = 3;
        break;
      case kDefault:
      default:
        ledEntryValue = 0;
    }
    ledControlEntry.setNumber(ledEntryValue);
    extraLedDio.ifPresent(out -> out.set(controlMode == VisionLEDMode.kOn));
  }

  @Override
  public void setPipeline(int pipeline) {
    pipeControlEntry.setNumber(pipeline);
  }

  @Override
  public void takeSnapshot() {
    setSnapshotState(0);
    setSnapshotState(1);
    snapshotCycles = 10;
  }

  private void setSnapshotState(int state) {
    snapshotEntry.setNumber(state);
  }

  @Override
  public void periodic() {
    curTarget =
        new LimelightTarget(
            xEntry.getDouble(0),
            yEntry.getDouble(0),
            areaEntry.getDouble(0),
            skewEntry.getDouble(0));
    snapshotCycles--;
    if (snapshotCycles == 0) {
      setSnapshotState(0);
    }
    snapshotCycles = Math.max(snapshotCycles, -1);
  }

  @Override
  public void updateSmartDashboard() {
    super.updateSmartDashboard();
    SmartDashboard.putNumber(name + "-X", curTarget.getX().getDegrees());
    SmartDashboard.putNumber(name + "-Y", curTarget.getY().getDegrees());
    SmartDashboard.putBoolean(name + "-HasTarget", hasTargets());
    SmartDashboard.putNumber(name + "-Area", curTarget.getArea());
    SmartDashboard.putNumber(
        name + "-EstimatedDistance",
        getEstimatedDistance(curTarget, FieldConstants.TARGET_HEIGHT_METERS));
    // SmartDashboard.putNumber(name + "-Pipe", getRunningPipeline());
  }

  @Override
  public void onStart() {
    setLed(VisionLEDMode.kOn);
  }

  @Override
  public void onStop() {
    setLed(VisionLEDMode.kOff);
  }

  @Override
  public void setUpTrackables() {
    super.setUpTrackables();
    logger.addTrackable(() -> curTarget.getX().getDegrees(), name + "_X", 10);
    logger.addTrackable(() -> curTarget.getY().getDegrees(), name + "_Y", 10);
    logger.addTrackable(
        () -> getEstimatedDistance(curTarget, FieldConstants.TARGET_HEIGHT_METERS),
        name + "_EstimatedDistance",
        10);
  }
}
