package frc.lib;

import frc.lib.logging.Logger;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

public class PreferencesParser {

  private FileReader fileReader;
  private JSONObject json;
  private JSONParser jsonParser;
  private Path preferencesPath = Paths.get(System.getProperty("user.home"), "Preferences.json");
  private String preferencesPathString = preferencesPath.toString();
  private final Logger logger;

  public PreferencesParser(Logger logger) {
    jsonParser = new JSONParser();
    update();
    this.logger = logger;
  }

  public String getString(String key) throws PreferenceDoesNotExistException {
    update();
    if (keyExists(key)) {
      return (String) json.get(key);
    }

    System.out.println("Key " + key + " does not exist in preferences!");
    logger.writeRaw("Key " + key + " does not exist in preferences!");
    throw new PreferenceDoesNotExistException(key);
  }

  public String tryGetString(String key, String defaultString) {
    try {
      return getString(key);
    } catch (PreferenceDoesNotExistException e) {
      return defaultString;
    }
  }

  public boolean getBoolean(String key) throws PreferenceDoesNotExistException {
    update();
    if (keyExists(key)) {
      return Boolean.parseBoolean(json.get(key).toString());
    }

    System.out.println("Key " + key + " does not exist in preferences!");
    logger.writeRaw("Key " + key + " does not exist in preferences!");
    throw new PreferenceDoesNotExistException(key);
  }

  public int getInt(String key) throws PreferenceDoesNotExistException {
    update();
    if (keyExists(key)) {
      return ((Long) json.get(key)).intValue();
    }

    System.out.println("Key " + key + " does not exist in preferences!");
    logger.writeRaw("Key " + key + " does not exist in preferences!");
    throw new PreferenceDoesNotExistException(key);
  }

  public int tryGetInt(String key, int defaultInt) {
    try {
      return getInt(key);
    } catch (PreferenceDoesNotExistException e) {
      return defaultInt;
    }
  }

  public double getDouble(String key) throws PreferenceDoesNotExistException {
    update();
    if (keyExists(key)) {
      return ((Number) json.get(key)).doubleValue();
    }

    System.out.println("Key " + key + " does not exist in preferences!");
    logger.writeRaw("Key " + key + " does not exist in preferences!");
    throw new PreferenceDoesNotExistException(key);
  }

  public double tryGetDouble(String key, double defaultDouble) {
    try {
      return getDouble(key);
    } catch (PreferenceDoesNotExistException e) {
      return defaultDouble;
    }
  }

  public boolean tryGetBoolean(String key, boolean defaultBoolean) {
    try {
      return getBoolean(key);
    } catch (PreferenceDoesNotExistException e) {
      return defaultBoolean;
    }
  }

  private boolean keyExists(String key) {
    if (json.get(key) == null) {
      return false;
    }

    return true;
  }

  public JSONObject getJSONObject(String key) throws PreferenceDoesNotExistException {
    update();
    if (keyExists(key)) {
      return (JSONObject) json.get(key);
    }

    System.out.println("Key " + key + " does not exist in preferences!");
    logger.writeRaw("Key " + key + " does not exist in preferences!");
    throw new PreferenceDoesNotExistException(key);
  }

  public JSONObject tryGetJSONObject(String key, JSONObject defaultObject) {
    try {
      return getJSONObject(key);
    } catch (PreferenceDoesNotExistException e) {
      return defaultObject;
    }
  }

  private void update() {
    try {
      fileReader = new FileReader(preferencesPathString, Charset.forName("UTF-8"));

      Object obj = jsonParser.parse(fileReader);
      json = (JSONObject) obj;

      fileReader.close();
    } catch (FileNotFoundException e) {
      System.out.println("Could not find preferences file at " + preferencesPathString);
      logger.writeRaw("Could not find preferences file at " + preferencesPathString);
    } catch (IOException e) {
      System.out.println("IOException in PreferencesParser");
      logger.writeRaw("IOException in PreferencesParser");
    } catch (ParseException e) {
      e.printStackTrace();
    }
  }
}
