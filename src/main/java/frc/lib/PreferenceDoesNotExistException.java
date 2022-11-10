package frc.lib;

public class PreferenceDoesNotExistException extends RuntimeException {
  private static final long serialVersionUID = 7609312699826252204L;
  public final String key;

  public PreferenceDoesNotExistException(String key) {
    super("Preferences key " + key + " does not exist!");
    this.key = key;
  }
}
