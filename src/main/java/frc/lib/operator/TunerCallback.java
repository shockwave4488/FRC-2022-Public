package frc.lib.operator;

import java.util.Map;

public interface TunerCallback {
  public void update(Map<String, Double> vals);
}
