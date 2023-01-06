package frc.robot.subsystems.c2022;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.PreferencesParser;
import frc.lib.logging.Logger;
import frc.lib.sensors.BeamBreak;
import frc.lib.wpiextensions.ShockwaveSubsystemBase;

public class Indexer extends ShockwaveSubsystemBase {

  // private final Tuner tuner;
  private double p;
  private double i;
  private double d;
  private double f;

  private static final double TARGET_RPM = 750; // 1000
  private static final double SHOOTING_TARGET_RPM = 1000; // 2000
  private static final double FENDER_TARGET_RPM = 750;
  private static final double REVERSE_RPM = -2000; // -2000
  private static final double PULLEY_RATIO =
      1.5; // real pulley ratio is 3.5 but we tuned everything for when it was ~1.53
  private static final double ENCODER_TICKS_PER_ROTATION = 2048;

  static enum Mode {
    Reverse(REVERSE_RPM),
    Off(0),
    Forward(TARGET_RPM),
    ForwardForFender(FENDER_TARGET_RPM),
    ForwardToFlywheel(SHOOTING_TARGET_RPM);

    double velocity;

    private Mode(double velocity) {
      this.velocity = velocity;
    }
  }

  public static enum IndexerState {
    NoBalls,
    LoadingFirst,
    OneBall,
    LoadingToFlywheel,
    TwoBalls,
    Flushing
  }

  private final StateSupplier state = new StateSupplier();
  private IndexerState indexerState = IndexerState.OneBall;
  private Mode conveyorState = Mode.Off;
  private final BeamBreak entranceBeamBreak;
  private final BeamBreak middleBeamBreak;
  private final BeamBreak flywheelBeamBreak;
  private final TalonFX conveyor;
  private final Logger logger;
  private final PreferencesParser prefs;

  public class StateSupplier {
    public IndexerState getState() {
      return indexerState;
    }

    public Mode getConveyorMode() {
      return conveyorState;
    }

    public boolean getEntranceBeamBreak() {
      return entranceBeamBreak.get();
    }

    public boolean getMiddleBeamBreak() {
      return middleBeamBreak.get();
    }

    public boolean getFlywheelBeamBreak() {
      return flywheelBeamBreak.get();
    }
  }

  public Indexer(
      int ConveyorID,
      int EntranceBBID,
      int MiddleBBID,
      int FlywheelBBID,
      Logger logger,
      PreferencesParser prefs) {
    conveyor = new TalonFX(ConveyorID);
    conveyor.configFactoryDefault();
    conveyor.setInverted(InvertType.InvertMotorOutput);
    conveyor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    conveyor.setNeutralMode(NeutralMode.Brake);
    conveyor.configVoltageCompSaturation(12);
    conveyor.enableVoltageCompensation(true);

    entranceBeamBreak = new BeamBreak(EntranceBBID);
    middleBeamBreak = new BeamBreak(MiddleBBID);
    flywheelBeamBreak = new BeamBreak(FlywheelBBID);

    this.logger = logger;
    this.prefs = prefs;

    updateFromPrefs();

    /*
    tuner = new Tuner(this::updateTuner, 2, prefs);

    tuner.addValueFromPrefs("IndexerP", 0);
    tuner.addValueFromPrefs("IndexerI", 0);
    tuner.addValueFromPrefs("IndexerD", 0);
    tuner.addValueFromPrefs("IndexerF", 0);

    tuner.start();
    */
  }

  /*
  public void updateTuner(Map<String, Double> vals) {
    p = vals.get("IndexerP");
    i = vals.get("IndexerI");
    d = vals.get("IndexerD");
    f = vals.get("IndexerF");

    updatePIDF(p, i, d, f);
  }
  */

  private synchronized void updatePIDF(double p, double i, double d, double f) {
    conveyor.config_kF(0, f);
    conveyor.config_kP(0, p);
    conveyor.config_kI(0, i);
    conveyor.config_kD(0, d);
  }

  public void updateFromPrefs() {
    p = prefs.tryGetValue(prefs::getDouble, "IndexerP", 0.0);
    i = prefs.tryGetValue(prefs::getDouble, "IndexerI", 0.0);
    d = prefs.tryGetValue(prefs::getDouble, "IndexerD", 0.0);
    f = prefs.tryGetValue(prefs::getDouble, "IndexerF", 0.0);

    updatePIDF(p, i, d, f);
  }

  public StateSupplier getIndexerStates() {
    return state;
  }

  public void recordState(IndexerState state) {
    indexerState = state;
  }

  @Override
  public void onStart() {
    conveyorState = Mode.Off;
  }

  @Override
  public void onStop() {
    conveyorState = Mode.Off;
  }

  @Override
  public void zeroSensors() {}

  public void spinToShoot() {
    conveyorState = Mode.ForwardToFlywheel;
  }

  public void spinToFenderShoot() {
    conveyorState = Mode.ForwardForFender;
  }

  public void spinForward() {
    conveyorState = Mode.Forward;
  }

  public void spinBackwards() {
    conveyorState = Mode.Reverse;
  }

  public void spinHold() {
    conveyorState = Mode.Off;
  }

  @Override
  public void periodic() {
    double rate = conveyorState.velocity / 600 * ENCODER_TICKS_PER_ROTATION * PULLEY_RATIO;
    if (conveyorState == Mode.ForwardForFender) {
      conveyor.set(ControlMode.Velocity, rate);
    } else if (conveyorState == Mode.ForwardToFlywheel) {
      conveyor.set(ControlMode.PercentOutput, 0.5);
    } else if (rate > 0) {
      conveyor.set(ControlMode.PercentOutput, 0.65);
    } else if (rate < 0) {
      conveyor.set(ControlMode.PercentOutput, -0.65);
    } else {
      conveyor.set(ControlMode.PercentOutput, 0);
    }

    if (!(conveyorState == Mode.Forward || conveyorState == Mode.Off)) {
      indexerState = IndexerState.Flushing;
    }
  }

  @Override
  public void updateSmartDashboard() {
    /*
    double rpm = Math.round(conveyor.getSelectedSensorVelocity());
    rpm *= PULLEY_RATIO * 600 / ENCODER_TICKS_PER_ROTATION;
    SmartDashboard.putNumber("Indexer Conveyor RPM", rpm);
    */
    SmartDashboard.putBoolean("Indexer Entrance bb tripped", state.getEntranceBeamBreak());
    SmartDashboard.putBoolean("Indexer Middle bb tripped", state.getMiddleBeamBreak());
    SmartDashboard.putBoolean("Indexer Flywheel bb tripped", state.getFlywheelBeamBreak());
    SmartDashboard.putString("Indexer Subsystem Mode", conveyorState.toString());
    IndexerState numBalls = state.getState();
    SmartDashboard.putBoolean(
        "One Ball", numBalls != IndexerState.NoBalls && numBalls != IndexerState.Flushing);
    SmartDashboard.putBoolean(
        "Two Balls",
        numBalls == IndexerState.TwoBalls || numBalls == IndexerState.LoadingToFlywheel);
  }

  @Override
  public void setUpTrackables() {
    logger.addTrackable(() -> state.getEntranceBeamBreak() ? 1 : 0, "EntranceBeamBreak", 10);
    logger.addTrackable(() -> state.getMiddleBeamBreak() ? 1 : 0, "MiddleBeamBreak", 10);
    logger.addTrackable(() -> state.getFlywheelBeamBreak() ? 1 : 0, "FlywheelBeamBreak", 10);
    // above trackables have higher frequencies to prevent the logger missing the beam breaks' quick
    // on/off changes
    logger.addTrackable(() -> conveyorState.velocity, "ConveyorState", 4);
    logger.addTrackable(
        () ->
            (Math.round(conveyor.getSelectedSensorVelocity())
                * PULLEY_RATIO
                * 600
                / ENCODER_TICKS_PER_ROTATION),
        "ConveyorRPM",
        4);
  }
}
