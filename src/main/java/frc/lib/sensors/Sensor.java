package frc.lib.sensors;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;

public abstract class Sensor {

  public enum Type {
    Analog,
    Digital;
  }

  private Type type;
  private AnalogInput analog;
  private DigitalInput digital;

  public Sensor(int port, Type type) {
    this.type = type;

    if (type == Type.Analog) {
      analog = new AnalogInput(port);
    } else {
      digital = new DigitalInput(port);
    }
  }

  public Object getObjectValue() {
    return (type == Type.Analog ? analog.getValue() : digital.get());
  }

  public abstract void reset();

  public abstract void loop();
}
