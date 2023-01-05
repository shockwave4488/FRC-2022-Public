package frc.lib.util.app;

import java.lang.reflect.Field;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import org.json.simple.JSONObject;

/** Contains basic functions that are used often. */
public final class Util {
  /** Prevent this class from being instantiated. */
  private Util() {}

  public static final double kEpsilon = 1e-12;

  /** Limits the given input to the given magnitude. */
  public static double limit(double v, double maxMagnitude) {
    return limit(v, -maxMagnitude, maxMagnitude);
  }

  public static double limit(double v, double min, double max) {
    return Math.min(max, Math.max(min, v));
  }

  public static String joinStrings(String delim, List<?> strings) {
    StringBuilder sb = new StringBuilder();
    for (int i = 0; i < strings.size(); ++i) {
      sb.append(strings.get(i).toString());
      if (i < strings.size() - 1) {
        sb.append(delim);
      }
    }
    return sb.toString();
  }

  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, kEpsilon);
  }

  public static boolean allCloseTo(List<Double> list, double value, double epsilon) {
    boolean result = true;
    for (Double value_in : list) {
      result &= epsilonEquals(value_in, value, epsilon);
    }
    return result;
  }

  public static double interpolate(double a, double b, double x) {
    x = limit(x, 0.0, 1.0);
    return a + (b - a) * x;
  }

  @SuppressWarnings("unchecked")
  public static <T> T toObj(JSONObject json, Class<T> clazz) {
    try {
      T output = clazz.getConstructor().newInstance();
      for (Object obj : json.entrySet()) {
        Map.Entry<String, Object> entry = (Map.Entry<String, Object>) obj;
        Field field = clazz.getField(entry.getKey());
        Object value = entry.getValue();
        Class<?> type = field.getType();
        if (type.isEnum()) {
          final String finalValue = (String) value;
          value =
              Arrays.stream(type.getEnumConstants())
                  .filter(enumConst -> ((Enum<?>) enumConst).name().equalsIgnoreCase(finalValue))
                  .findFirst()
                  .orElseThrow();
        } else if (value instanceof JSONObject) {
          value = toObj((JSONObject) value, type);
        }
        field.set(output, value);
      }
      return output;
    } catch (Exception e) {
      throw new RuntimeException("Unable to get obj from JSONObject", e);
    }
  }
}
