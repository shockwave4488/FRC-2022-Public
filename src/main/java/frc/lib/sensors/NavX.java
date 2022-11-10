package frc.lib.sensors;

import com.kauailabs.navx.AHRSProtocol.AHRSUpdateBase;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.ITimestampedDataSubscriber;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

/** Driver for a NavX board. Basically a wrapper for the {@link AHRS} class */
public class NavX {
  protected class Callback implements ITimestampedDataSubscriber {
    @Override
    public void timestampedDataReceived(
        long system_timestamp, long sensor_timestamp, AHRSUpdateBase update, Object context) {
      synchronized (NavX.this) {
        // This handles the fact that the sensor is inverted from our coordinate conventions.
        if (mLastSensorTimestampMs != kInvalidTimestamp
            && mLastSensorTimestampMs < sensor_timestamp) {
          mYawRateDegreesPerSecond =
              1000.0
                  * (-mYawDegrees - update.yaw)
                  / (double) (sensor_timestamp - mLastSensorTimestampMs);
          mPitchRateDegreesPerSecond =
              1000.0
                  * (-mPitchDegrees - update.pitch)
                  / (double) (sensor_timestamp - mLastSensorTimestampMs);
        }
        mLastSensorTimestampMs = sensor_timestamp;
        mYawDegrees = -update.yaw;
        mPitchDegrees = -update.pitch;
      }
    }
  }

  protected AHRS mAHRS;
  protected Rotation2d mYawAdjustment = new Rotation2d();
  protected Rotation2d mPitchAdjustment = new Rotation2d();
  protected double mYawDegrees;
  protected double mYawRateDegreesPerSecond;
  protected double mPitchDegrees;
  protected double mPitchRateDegreesPerSecond;
  protected static final long kInvalidTimestamp = -1;
  protected long mLastSensorTimestampMs;

  public NavX(SPI.Port spi_port_id) {
    mAHRS = new AHRS(spi_port_id, (byte) 200);
    resetState();
    mAHRS.registerCallback(new Callback(), null);
    mAHRS.enableBoardlevelYawReset(true);
  }

  public synchronized void reset() {
    mAHRS.reset();
    resetState();
  }

  public boolean isCalibrating() {
    return mAHRS.isCalibrating();
  }

  public synchronized void zeroYaw() {
    mAHRS.zeroYaw();
    resetState();
  }

  private void resetState() {
    mLastSensorTimestampMs = kInvalidTimestamp;
    mYawDegrees = 0.0;
    mYawRateDegreesPerSecond = 0.0;
    mPitchDegrees = 0.0;
    mPitchRateDegreesPerSecond = 0.0;
  }

  public synchronized void setYawAdjustment(Rotation2d adjustment) {
    mYawAdjustment = adjustment;
  }

  public synchronized void setPitchAdjustment(Rotation2d adjustment) {
    mPitchAdjustment = adjustment;
  }

  protected synchronized double getRawYawDegrees() {
    return mYawDegrees;
  }

  protected synchronized double getRawPitchDegrees() {
    return mPitchDegrees;
  }

  public Rotation2d getYaw() {
    return mYawAdjustment.rotateBy(Rotation2d.fromDegrees(getRawYawDegrees()));
  }

  public double getYawRateDegreesPerSec() {
    return mYawRateDegreesPerSecond;
  }

  public double getYawRateRadiansPerSec() {
    return 180.0 / Math.PI * getYawRateDegreesPerSec();
  }

  public Rotation2d getPitch() {
    return mPitchAdjustment.rotateBy(Rotation2d.fromDegrees(getRawPitchDegrees()));
  }

  public double getPitchRateDegreesPerSec() {
    return mPitchRateDegreesPerSecond;
  }

  public double getRawAccelX() {
    return mAHRS.getRawAccelX();
  }
}
