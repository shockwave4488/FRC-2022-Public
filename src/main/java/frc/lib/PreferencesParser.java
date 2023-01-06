package frc.lib;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.logging.Logger;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.function.Function;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

public class PreferencesParser {
  private final JSONObject json;
  private final Logger logger;
  private static final String PREFS_STRING = "Preferences.json";
  private static final String PREFS_DIR_FILE_STRING = "configDir.txt";

  public PreferencesParser(Logger logger) {
    this.logger = logger;
    JSONParser jsonParser = new JSONParser();
    String preferencesPathString;

    boolean robotIsReal = RobotBase.isReal();

    Path baseDir = Filesystem.getOperatingDirectory().toPath();
    Path prefsDir =
        baseDir.resolve(
            (!robotIsReal ? "simulation" + File.separator : "") + PREFS_DIR_FILE_STRING);

    String prefsDirString = "";
    try {
      prefsDirString = Files.readString(prefsDir);
    } catch (IOException e) {
      System.out.print(
          "Could not find preferences directory from "
              + prefsDir.toAbsolutePath().toString()
              + ". ");
      if (robotIsReal) {
        System.out.println(
            "Did you forget to send a 'configDir.txt' file containing the prefs location to the RoboRIO's home directory?");
      } else {
        System.out.println(
            "Did you forget to run './gradlew simulate' to specify the prefs location?");
      }
    }

    if (robotIsReal) {
      preferencesPathString =
          Filesystem.getDeployDirectory()
              .toPath()
              .resolve(prefsDirString + File.separator + PREFS_STRING)
              .toString();
    } else {
      preferencesPathString = Path.of(prefsDirString, PREFS_STRING).toString();
    }

    JSONObject jsonTemp = new JSONObject();
    try (FileReader fileReader = new FileReader(preferencesPathString, Charset.forName("UTF-8"))) {
      Object obj = jsonParser.parse(fileReader);
      jsonTemp = (JSONObject) obj;
    } catch (FileNotFoundException e) {
      System.out.println("Could not find preferences file at " + preferencesPathString);
      logger.writeRaw("Could not find preferences file at " + preferencesPathString);
    } catch (IOException e) {
      System.out.println("IOException in PreferencesParser");
      logger.writeRaw("IOException in PreferencesParser");
    } catch (ParseException e) {
      e.printStackTrace();
    }

    json = jsonTemp;
  }

  private boolean keyExists(String key) {
    return json.get(key) != null;
  }

  public String getString(String key) throws PreferenceDoesNotExistException {
    if (keyExists(key)) {
      return (String) json.get(key);
    }

    throw new PreferenceDoesNotExistException(key, logger);
  }

  public boolean getBoolean(String key) throws PreferenceDoesNotExistException {
    if (keyExists(key)) {
      return Boolean.parseBoolean(json.get(key).toString());
    }

    throw new PreferenceDoesNotExistException(key, logger);
  }

  public int getInt(String key) throws PreferenceDoesNotExistException {
    if (keyExists(key)) {
      return ((Long) json.get(key)).intValue();
    }

    throw new PreferenceDoesNotExistException(key, logger);
  }

  public double getDouble(String key) throws PreferenceDoesNotExistException {
    if (keyExists(key)) {
      return ((Number) json.get(key)).doubleValue();
    }

    throw new PreferenceDoesNotExistException(key, logger);
  }

  public JSONObject getJSONObject(String key) throws PreferenceDoesNotExistException {
    if (keyExists(key)) {
      return (JSONObject) json.get(key);
    }

    throw new PreferenceDoesNotExistException(key, logger);
  }

  public <T> T tryGetValue(Function<String, T> valueGetter, String key, T defaultValue) {
    try {
      return valueGetter.apply(key);
    } catch (PreferenceDoesNotExistException e) {
      return defaultValue;
    }
  }
}
