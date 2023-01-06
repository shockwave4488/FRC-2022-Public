package frc.lib.sensors;

public class HallEffect extends Sensor {

  public HallEffect(int port) {
    super(port, Type.Digital);
  }

  public boolean get() {
    return !((boolean) getObjectValue());
  }

  @Override
  public void reset() {}

  @Override
  public void loop() {}
}
