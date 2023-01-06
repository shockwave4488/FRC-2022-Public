package frc.lib.logging;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.charset.Charset;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;

public class Logger {

  private String fullPath;
  private double initializedTimestamp;
  private boolean createdFiles;
  private BufferedWriter mainWriter;
  private ArrayList<Tracker> trackers = new ArrayList<Tracker>();
  private ArrayList<StringTracker> stringTrackers = new ArrayList<StringTracker>();
  // private Path loggingPath = Paths.get(System.getProperty("user.home"), "logs");
  private String startPath =
      Filesystem.getOperatingDirectory()
          + (!RobotBase.isReal() ? File.separator + "simulation" : "")
          + File.separator
          + "logs";
  private final SimpleDateFormat sdfdate = new SimpleDateFormat("yyyy-MM-dd");
  private final SimpleDateFormat sdftime = new SimpleDateFormat("HH-mm-ss-SSSS");
  private static final int FREQUENCY_CAP = 100;

  public Logger() {}

  public void createFiles() {
    String date = dateStamp();
    String matchType = DriverStation.getMatchType().name() + "_";
    String matchNumber = DriverStation.getMatchNumber() + "_";

    if (matchType.equals("None_")) {
      matchType = "";
      matchNumber = "";
    }

    fullPath =
        startPath
            + File.separator
            + date
            + File.separator
            + (matchType + matchNumber)
            + timeStamp();

    System.out.println("Logging full path: " + fullPath);
    System.out.println("Logging start of path: " + startPath);
    // System.out.println("Logging old start of path: " + loggingPath.toString());

    File directory = new File(fullPath);
    if (!directory.exists()) {
      boolean mkdrisSuccessful = directory.mkdirs();
      if (mkdrisSuccessful == false) {
        // TODO: What should we do if a logging directory and/or its parent directories aren't set
        // up correctly?
        System.out.println("LOGGER MKDRIS FAILED, SAY SOMETHING TO PROGRAMMING");
        System.out.println("LOGGER MKDRIS FAILED, SAY SOMETHING TO PROGRAMMING");
      }
    }

    try {
      mainWriter =
          new BufferedWriter(
              new FileWriter(fullPath + File.separator + "main.txt", Charset.forName("UTF-8")));
    } catch (IOException e) {
      e.printStackTrace();
    }

    for (Tracker tracker : trackers) tracker.createFile(fullPath);
    for (StringTracker tracker : stringTrackers) tracker.createFile(fullPath);

    createdFiles = true;
  }

  /**
   * Initializing before creating files is expensive, ~0.8 seconds Reinitializing will recreate
   * files, so it is also expensive
   */
  public void initialize() {
    System.out.println("Initializing logger");
    if (initialized()) {
      flush();
      closeWriters();
      createdFiles = false;
    }

    if (!createdFiles) {
      createFiles();
    }

    initializedTimestamp = Timer.getFPGATimestamp();
  }

  private void closeWriters() {
    try {
      mainWriter.close();
    } catch (IOException e) {
      e.printStackTrace();
    }

    for (Tracker tracker : trackers) tracker.closeWriter();
    for (StringTracker tracker : stringTrackers) tracker.closeWriter();
  }

  public boolean initialized() {
    return mainWriter != null;
  }

  public void update() {
    for (int counter = 0; counter < trackers.size(); counter++) {
      trackers.get(counter).update(secondStamp());
    }
    for (int counter = 0; counter < stringTrackers.size(); counter++) {
      stringTrackers.get(counter).update(secondStamp());
    }
  }

  /**
   * This method creates a file for a Trackable, which will log the given Trackable (double).
   *
   * @param target The double you want to log preceded by a lambda.
   * @param name The name of the file
   * @param frequency How many times a second the logger runs.
   */
  public void addTrackable(Trackable target, String name, int frequency) {
    name = name.replace(" ", "_");
    Tracker newTracker = new Tracker(target, name, frequency);
    trackers.add(newTracker);
  }

  /**
   * This method creates a file of StringTrackables, which logs Strings and sets of data. It also
   * takes a header to allow its logging files to follow CSV (comma separated value) format.
   *
   * @param target The list of data or string you want to log preceded by a lambda. Java will
   *     automatically convert doubles you put alongside strings to strings (which is ideally always
   *     the case because the doubles should be separated by commas)
   * @param name The name of the file
   * @param frequency How many times a second code is logged. Keep in mind that StringTrackables are
   *     performance heavy, so don't set a frequency any higher than necessary.
   * @param header The first line of the file, it should list what each value is in the order
   *     they're set in target
   */
  public void addStringTrackable(
      StringTrackable target, String name, int frequency, String header) {
    name = name.replace(" ", "_");
    if (header.isEmpty()) {
      header = "Timestamp (seconds),Message";
    }
    frequency = Math.min(frequency, FREQUENCY_CAP);
    StringTracker newStringTracker = new StringTracker(target, name, frequency, header);
    stringTrackers.add(newStringTracker);
  }

  public void writeToLogFormatted(Object caller, String message) {
    String callerName;
    if (caller == null) {
      callerName = "null";
    } else if (caller instanceof String) {
      callerName = (String) caller;
    } else {
      callerName = caller.getClass().getSimpleName();
    }

    if (!initialized()) {
      System.out.println("Writing to main log before initializing it!");
      return;
    }

    try {
      mainWriter.write(secondStamp() + "\t" + callerName + "\t" + message + "\n");
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  /**
   * Formatted version of logs that adheres to strategy programming standards. For use in mock build
   * week.
   *
   * @param callClass - the top level class that this function is being called from
   * @param routine - the current routine the robot is executing
   * @param system - the specific subsystem that is involved
   * @param message - the state
   */
  public void writeToLogFormatted(Object callClass, Object routine, Object system, Object message) {
    // Stores important information from object parameters
    if (callClass == null) {
      callClass = "null";
    } else if (!(callClass instanceof String)) {
      callClass = callClass.getClass().getSimpleName();
    }
    if (routine == null) {
      routine = "null";
    } else if (!(routine instanceof String)) {
      routine = routine.getClass().getSimpleName();
    }
    if (system == null) {
      system = "null";
    } else if (!(system instanceof String)) {
      system = system.getClass().getSimpleName();
    }
    if (message == null) {
      message = "null";
    } else if (!(system instanceof String)) {
      message = message.toString();
    }

    if (!initialized()) {
      System.out.println("Writing to main log before initializing it!");
      return;
    }

    try {
      mainWriter.write(
          secondStamp()
              + "\t"
              + callClass
              + "\t"
              + routine
              + "\t"
              + system
              + "\t"
              + message
              + "\n");
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  public void writeRaw(String message) {
    if (!initialized()) {
      System.out.println("Writing to main log before initializing it!");
      return;
    }

    try {
      mainWriter.write(message + "\n");
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  public void flush() {
    System.out.println("Flushing logs!");
    if (!initialized()) {
      System.out.println("Flushing logs before they are initialized!");
      return;
    }

    try {
      mainWriter.flush();
      for (Tracker tracker : trackers) {
        tracker.flush();
      }
      for (StringTracker stringTracker : stringTrackers) {
        stringTracker.flush();
      }
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  private String secondStamp() {
    double time = Timer.getFPGATimestamp() - initializedTimestamp;
    time = Math.round(time * 1000.0) / 1000.0;
    return Double.toString(time);
  }

  private String timeStamp() {
    Date date = new Date();
    String timeString = sdftime.format(date);
    return timeString;
  }

  private String dateStamp() {
    Date date = new Date();
    String dateString = sdfdate.format(date);
    return dateString;
  }
}
