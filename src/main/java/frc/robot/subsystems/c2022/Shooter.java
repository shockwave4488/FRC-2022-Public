package frc.robot.subsystems.c2022;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.PreferencesParser;
import frc.lib.controlsystems.SetPointProfile;
import frc.lib.devices.LinearServo;
import frc.lib.logging.Logger;
import frc.lib.util.app.Util;
import frc.lib.wpiextensions.ShockwaveSubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends ShockwaveSubsystemBase {

  private static final double PULLEY_RATIO = 1;
  private static final double ENCODER_TICKS_PER_ROTATION = 2048;
  private static final int MAX_HOOD_POSITION = 83;
  private static final double HOOD1OFFSET = 0;
  private static final double HOOD2OFFSET = 0; // RIGHT (battery side is front)
  private static final double MAX_SHOOTER_RPM = 4000;
  private static final double CURRENT_RPM_OFFSET = -15;
  private static final double CURRENT_HOOD_OFFSET = 20;

  // private final Tuner tuner;
  private final PreferencesParser prefs;
  private double p;
  private double i;
  private double d;
  private double f;

  private int countCycles = 0;
  private static final int MIN_COUNT_CYCLES = 6;
  private static final double ERROR_EPSILON = 100;
  private int minCountCycles;
  private double errorEpsilon;
  private double targetRPM = 0;

  private final Logger logger;
  private final TalonFX masterFlywheel; // master
  private final TalonFX followerFlywheel; // follower
  private final LinearServo hood1;
  private final LinearServo hood2;
  private double desiredHoodPosition = 40; // fender hood = 15, tarmac hood = 40
  private SetPointProfile YOffsetToRpm = new SetPointProfile();
  private SetPointProfile RPMToHoodLevel = new SetPointProfile();
  private SetPointProfile YOffsetToHoodLevel = new SetPointProfile();

  public Shooter(
      int MflywheelID,
      int FflywheelID,
      int PCM_ID,
      int hood1ID,
      int hood2ID,
      Logger logger,
      PreferencesParser prefs) {
    this.logger = logger;

    masterFlywheel = new TalonFX(MflywheelID);
    followerFlywheel = new TalonFX(FflywheelID);
    masterFlywheel.configFactoryDefault();
    followerFlywheel.configFactoryDefault();
    masterFlywheel.setInverted(InvertType.InvertMotorOutput);
    followerFlywheel.follow(masterFlywheel);
    followerFlywheel.setInverted(InvertType.OpposeMaster);
    masterFlywheel.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    hood1 = new LinearServo(hood1ID, 140, 17);
    hood2 = new LinearServo(hood2ID, 140, 17);
    // don't change max length from 140, the servo calculates its output based on the % returned by
    // (input/2nd argument)

    SmartDashboard.putNumber("Shooter RPM Offset", CURRENT_RPM_OFFSET);
    SmartDashboard.putNumber("Shooter Hood Offset", CURRENT_HOOD_OFFSET);

    setInterpolationPoints();

    this.prefs = prefs;
    updateFromPrefs();

    /*
    tuner = new Tuner(this::updateTuner, 2, prefs);

    tuner.addValueFromPrefs("ShooterP", 0);
    tuner.addValueFromPrefs("ShooterI", 0);
    tuner.addValueFromPrefs("ShooterD", 0);
    tuner.addValueFromPrefs("ShooterF", 0);

    tuner.start();
    */
  }

  /*
  public void updateTuner(Map<String, Double> vals) {
    p = vals.get("ShooterP");
    i = vals.get("ShooterI");
    d = vals.get("ShooterD");
    f = vals.get("ShooterF");

    updatePIDF(p, i, d, f);
  }
  */

  public void updateFromPrefs() {
    p = prefs.tryGetValue(prefs::getDouble, "ShooterP", 0.0);
    i = prefs.tryGetValue(prefs::getDouble, "ShooterI", 0.0);
    d = prefs.tryGetValue(prefs::getDouble, "ShooterD", 0.0);
    f = prefs.tryGetValue(prefs::getDouble, "ShooterF", 0.0);

    updatePIDF(p, i, d, f);
  }

  private synchronized void updatePIDF(double p, double i, double d, double f) {
    masterFlywheel.config_kF(0, f);
    masterFlywheel.config_kP(0, p);
    masterFlywheel.config_kI(0, i);
    masterFlywheel.config_kD(0, d);
  }

  private void setInterpolationPoints() {
    /*
    Old values (rpm is -50 to account for offset)
    YOffsetToRpm.add(4.2, 2150); // 3.18 m
    YOffsetToRpm.add(1.3, 2300); // 3.5 m
    YOffsetToRpm.add(-3.5, 2500); // 4.37 m
    YOffsetToRpm.add(-5.5, 2700); // 4.96 m

    RPMToHoodLevel.add(1850, 8.6);
    RPMToHoodLevel.add(1950, 22.3); // 2.45 m
    RPMToHoodLevel.add(2150, 32.9);
    RPMToHoodLevel.add(2300, 40.2);
    RPMToHoodLevel.add(2500, 55.1);
    RPMToHoodLevel.add(2700, 70.4);
    */

    // The actual rpm was often 50 higher than the desired rpms in the table below (an offset
    // doesn't need to be applied, just an fyi for if we fix that issue)
    // Keeping the below table in case we desperately need a new interpolation table for an event
    // that keeps cargo up to the correct PSI. If I recall correctly this table was made using some
    // higher PSI balls
    /*
    YOffsetToRpm.add(11.7, 2050);
    YOffsetToRpm.add(4.2, 2150); // 3.18 m
    YOffsetToRpm.add(1.2, 2225); // 3.5 m
    YOffsetToRpm.add(-3.5, 2425); // 4.37 m
    YOffsetToRpm.add(-5.5, 2550); // 4.96 m
    YOffsetToRpm.add(-7.3, 3000); // Side of launch pad
    */

    // Below table is done with balls that should reflect the psi of balls at competition
    // Used with the higher climber bars & limelight position (no low rung passage)
    // Commented meter values are from front of bumper (putting the back of laser distance device on
    // the front of the bumper) to the spot with the blue tape on the fender (the laser pointer
    // facing straight forward)
    /*
    YOffsetToRpm.add(10, 2200); //  1.25m
    YOffsetToRpm.add(4.2, 2295); //  1.77m
    YOffsetToRpm.add(1.2, 2370); //  2.22m
    YOffsetToRpm.add(-3.5, 2575); // hood input is 54.3 |    3.11m
    YOffsetToRpm.add(-5.5, 2675); // hood input is 59.8 |    3.68m
    YOffsetToRpm.add(-7.4, 2775); //  4.29m
    */
    // closest point is 11.7 and 1.45m away
    YOffsetToRpm.add(8.6, 2295);
    YOffsetToRpm.add(5.0, 2370);
    YOffsetToRpm.add(-0.6, 2575);
    YOffsetToRpm.add(-3.2, 2675);
    YOffsetToRpm.add(-5.3, 2775);

    // RPMToHoodLevel is currently unused except for SpinFlywheelDist which isn't used either.
    RPMToHoodLevel.add(1850, 8.6);
    RPMToHoodLevel.add(1950, 22.3); // 2.45 m
    RPMToHoodLevel.add(2150, 32.9);
    RPMToHoodLevel.add(2225, 40);
    RPMToHoodLevel.add(2425, 54);
    RPMToHoodLevel.add(2550, 70); // 70 is max hood angle
    RPMToHoodLevel.add(3000, 65); // Side of launch pad

    // Above YOffsetToRPM table was done with this
    YOffsetToHoodLevel.add(8.6, 33);
    YOffsetToHoodLevel.add(-5.3, 65);
    // Above line from YOffsetToHoodLevel has a slope of -2.75 (in case you need to graph it to find
    // a precise point)
  }

  public double getRPMFromYOffset(double yOffset) {
    return YOffsetToRpm.get(yOffset);
  }

  public double getHoodLevelFromRPM(double RPM) {
    return RPMToHoodLevel.get(RPM);
  }

  public double getHoodLevelFromYOffset(double yOffset) {
    return YOffsetToHoodLevel.get(yOffset);
  }

  public void setRPM(double RPM) {
    targetRPM = RPM;
    countCycles = 0;
    minCountCycles = MIN_COUNT_CYCLES;
    errorEpsilon = ERROR_EPSILON;
  }

  public void setRPM(double RPM, int minCountCycles, double epsilon) {
    targetRPM = RPM;
    countCycles = 0;
    this.minCountCycles = minCountCycles;
    errorEpsilon = epsilon;
  }

  public void setRPMOffset(double offset) {
    SmartDashboard.putNumber("Shooter RPM Offset", offset);
  }

  public double getRPM() {
    double curRate = masterFlywheel.getSelectedSensorVelocity();
    return curRate * 600 * PULLEY_RATIO / ENCODER_TICKS_PER_ROTATION;
  }

  public double getRPMOffset() {
    return SmartDashboard.getNumber("Shooter RPM Offset", CURRENT_RPM_OFFSET);
  }

  public double getHoodOffset() {
    return SmartDashboard.getNumber("Shooter Hood Offset", CURRENT_HOOD_OFFSET);
  }

  public boolean isReady() {
    return (countCycles >= minCountCycles) && hoodReady();
  }

  public void doneCycle() {
    if (Util.epsilonEquals(getRPM(), targetRPM, errorEpsilon)) {
      countCycles++;
    } else {
      countCycles = 0;
    }
  }

  public void setHoodPosition(double hoodPosition) {
    desiredHoodPosition =
        Math.max(Math.min(MAX_HOOD_POSITION, hoodPosition), ShooterConstants.MIN_HOOD_POSITION);
  }

  public double getHoodPosition() {
    return desiredHoodPosition;
  }

  private boolean hoodReady() {
    return (hood1.isFinished()) && (hood2.isFinished());
  }

  public void logShooterValues(double yOffset, double RPM, double hoodLevel) {
    logger.writeToLogFormatted(
        this, "Shot Log: yOffset: " + yOffset + "  RPM: " + RPM + "  hoodLevel: " + hoodLevel);
  }

  public void stop() {
    targetRPM = 0;
  }

  @Override
  public void onStart() {
    stop();
  }

  @Override
  public void periodic() {
    targetRPM = Math.min(MAX_SHOOTER_RPM, targetRPM);
    double rate = targetRPM / 600 * ENCODER_TICKS_PER_ROTATION / PULLEY_RATIO; // Ticks per 100ms
    if (rate != 0) {
      masterFlywheel.set(ControlMode.Velocity, rate);
    } else {
      // Let flywheel coast instead of forcing it to a rpm of 0
      masterFlywheel.set(ControlMode.PercentOutput, 0);
    }
    hood1.updateCurPos();
    hood2.updateCurPos();
    hood1.setPosition(desiredHoodPosition + HOOD1OFFSET);
    hood2.setPosition(desiredHoodPosition + HOOD2OFFSET);

    doneCycle();
  }

  @Override
  public void onStop() {
    stop();
  }

  @Override
  public void zeroSensors() {}

  @Override
  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Shooter Target Speed", targetRPM);
    SmartDashboard.putNumber("Shooter Actual Speed", getRPM());
    SmartDashboard.putNumber("Desired Shooter Hood Position", desiredHoodPosition);
    // SmartDashboard.putBoolean("Shooter is Ready", isReady());
    // SmartDashboard.putNumber("Hood 1 Position", hood1.getPosition());
    // SmartDashboard.putNumber("Hood 2 Position", hood2.getPosition());
  }

  @Override
  public void setUpTrackables() {
    logger.addTrackable(() -> targetRPM, "ShooterFlywheelDesiredRPM", 5);
    logger.addTrackable(() -> getRPM(), "ShooterFlywheelActualRPM", 5);
    logger.addTrackable(() -> desiredHoodPosition, "ShooterDesiredHoodPosition", 5);
    logger.addTrackable(() -> isReady() ? 1 : 0, "ShooterIsReady", 5);
  }
}
