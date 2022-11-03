package frc.lib.devices;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;

public class LinearServo extends Servo {
  double m_speed;
  double m_length;
  double setPos;
  double curPos;
  boolean finished;
  double lastTime;
  /**
   * Parameters for L16-R Actuonix Linear Actuators
   *
   * @param channel PWM channel used to control the servo
   * @param length max length of the servo [mm]
   * @param speed max speed of the servo [mm/second]
   */
  public LinearServo(int channel, int length, int speed) {
    super(channel);
    setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    m_length = length;
    m_speed = speed;
    lastTime = 0;
    /*setting a linear servo looks like: (name of servo) = new LinearServo(PWM, length (max distance), 32 (max speed));
    Check the values by looking up the servo model in
    https://wcproducts.info/files/frc/manuals/WCP%20Miniature%20Linear%20Servo%20Actuators%20-%20User%20Guide.pdf
    (the product manual)
    */
  }

  /**
   * Run this method in any periodic function to update the position estimation of your servo
   *
   * @param setpoint the target position of the servo [mm]
   */
  public void setPosition(double setpoint) {
    lastTime = Timer.getFPGATimestamp();
    setPos = MathUtil.clamp(setpoint, 0, m_length);
    setSpeed((setPos / m_length * 2) - 1);
  }

  /** Run this method in any periodic function to update the position estimation of your servo */
  public void updateCurPos() {
    double currentTime = Timer.getFPGATimestamp();
    double dt = currentTime - lastTime;
    lastTime = currentTime;
    if (curPos > setPos + m_speed * dt) {
      curPos -= m_speed * dt;
    } else if (curPos < setPos - m_speed * dt) {
      curPos += m_speed * dt;
    } else {
      curPos = setPos;
    }
  }

  /**
   * Current position of the servo, must be calling {@link #updateCurPos() updateCurPos()}
   * periodically
   *
   * @return Servo Position [mm]
   */
  public double getPosition() {
    return curPos;
  }

  /**
   * Checks if the servo is at its target position, must be calling {@link #updateCurPos()
   * updateCurPos()} periodically
   *
   * @return true when servo is at its target
   */
  public boolean isFinished() {
    finished = (Math.abs(curPos - setPos) < .0000001);
    return finished;
  }
}
