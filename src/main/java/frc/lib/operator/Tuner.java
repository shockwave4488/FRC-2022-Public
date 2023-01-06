package frc.lib.operator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.PreferencesParser;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;

public class Tuner {

  private double period;
  private AtomicBoolean shouldRun = new AtomicBoolean(false);
  private Thread updateThread;
  private PreferencesParser prefs;
  private Map<String, Double> vals = new HashMap<String, Double>();

  /**
   * Create a new Tuner. SmartDashboard values will be collected [frequency] times a second, and if
   * any are changed, they will be passed to the callback.
   *
   * <p>Note that using SmartDashboard.putNumber() and SmartDashboard.getNumber() is easier than
   * making a Tuner. This class will probably be removed after our 2022 season.
   *
   * @param callback callback to call when a value is changed
   * @param frequency how many times to check SmartDashboard each second
   */
  public Tuner(TunerCallback callback, int frequency, PreferencesParser prefs) {
    this.prefs = prefs;

    period = 1 / (double) frequency;

    updateThread =
        new Thread(
            new Runnable() {
              public void run() {
                callback.update(vals);
                while (shouldRun.get()) {
                  Map<String, Double> newVals = getVals();
                  for (Map.Entry<String, Double> entry : newVals.entrySet()) {
                    if (entry.getValue() != vals.get(entry.getKey()).doubleValue()) {
                      callback.update(newVals);
                    }
                  }
                  vals = newVals;

                  try {
                    Thread.sleep(Math.round(period * 1000));
                  } catch (InterruptedException e) {
                    e.printStackTrace();
                  }
                }
              }
            });
  }

  /**
   * Add a new key to track, and put it on SmartDashboard
   *
   * @param key key to track
   * @param initialValue The initial value to put onto SmartDashboard
   */
  public void addValue(String key, double initialValue) {
    vals.put(key, initialValue);
    SmartDashboard.putNumber(key, initialValue);
  }

  /**
   * Add a value using the key's prefs value as the initial value
   *
   * @param key the key to track and get from prefs
   * @param defaultValue default value to use in case the pref doesnt exist
   */
  public void addValueFromPrefs(String key, double defaultValue) {
    double val = prefs.tryGetValue(prefs::getDouble, key, defaultValue);
    addValue(key, val);
  }

  private Map<String, Double> getVals() {
    Map<String, Double> newVals = new HashMap<String, Double>();
    for (Map.Entry<String, Double> entry : vals.entrySet()) {
      String entryKey = entry.getKey();
      Double entryValue = entry.getValue();
      double val = SmartDashboard.getNumber(entryKey, entryValue);
      newVals.put(entryKey, val);
    }
    return newVals;
  }

  /** Start the thread the tuner runs on. The callback will be immediately called once */
  public void start() {
    shouldRun.set(true);
    updateThread.start();
  }

  /** Stop the tuner thread */
  public void stop() {
    shouldRun.set(false);
  }
}
