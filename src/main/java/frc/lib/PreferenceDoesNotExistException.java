package frc.lib;

import frc.lib.logging.Logger;

public class PreferenceDoesNotExistException extends RuntimeException {
  private static final long serialVersionUID = 7609312699826252204L;
  public final String key;

  public PreferenceDoesNotExistException(String key, Logger logger) {
    this(key);
    logger.writeRaw("Key " + key + " does not exist in preferences!");
  }

  public PreferenceDoesNotExistException(String key) {
    super("Preferences key " + key + " does not exist!");
    this.key = key;
    System.out.println("Key " + key + " does not exist in preferences!");
  }
}
