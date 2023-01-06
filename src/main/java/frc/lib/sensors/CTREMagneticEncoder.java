package frc.lib.sensors;

import edu.wpi.first.wpilibj.Counter;

/**
 * Note that foundation of this class was obtained from Team 3128 Aluminum Narwhals's GitHub
 * repository at the following link:
 * https://github.com/Team3128/3128-robot-2021/blob/main/src/main/java/org/team3128/common/hardware/encoder/both/CTREMagneticEncoder.java
 *
 * <p>Driver for a CTRE Magnetic Encoder using DIO ports on the roborio.
 *
 * <p>When instantiated, it sets the quadrature distance from the absolute angle. So, between 0 and
 * 1 rotations. When reset, the distance goes to zero.
 *
 * <p>Internally, it uses a Counter to measure the PWM.
 *
 * @author Narwhal, modified by Shockwave
 */
public class CTREMagneticEncoder {

  private Counter pwmCounter;
  private double offset = 0;

  /**
   * @param pwmPort DIO port connected to pin 9 on the encoder, the PWM pin
   * @param offset The offest you want to apply to the encoder, put 0 if you don't know or don't
   *     want one
   * @param inverted whether or not the encoder is inverted
   */
  public CTREMagneticEncoder(
      int pwmPort, double pulsesPerRevolution, double offset, boolean inverted) {
    this.offset = offset;
    pwmCounter = new Counter(pwmPort);
    pwmCounter.setSemiPeriodMode(true);

    // wait for the pwm signal to be counted
    try {
      Thread.sleep(5);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }

  /**
   * Gets the angle in radians without an offset
   *
   * @return Angle in radians
   */
  public double getAngle() {
    // from 1 to 4096 us
    return ((pwmCounter.getPeriod() - 1e-6) / 4095e-6) * 2 * Math.PI;
  }

  /**
   * Gets the angle of the encoder in radians with the offset passed through the constructor
   *
   * @return Adjusted angle in radians
   */
  public double getAngleOffset() {
    return (getAngle() - offset + 2 * Math.PI) % (2 * Math.PI);
  }

  /**
   * Gets the raw value of the encoder
   *
   * @return Returns the raw value of the encoder in ticks
   */
  public double getRawValue() {
    return pwmCounter.getPeriod() * 1e6;
  }
}
