package frc.lib.wpiextensions;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Insert comment on what exactly this class does */
public abstract class ShockwaveSubsystemBase extends SubsystemBase {
  /**
   * What the subsystem should do upon starting up, replacement of onStart() from our old Loop class
   */
  public abstract void onStart();

  /**
   * What the subsystem should do upon shutting down, replacement of onStop() from our old Loop
   * class
   */
  public abstract void onStop();

  /** Where all sensors should be zeroed/reset */
  public abstract void zeroSensors();

  /** Where you should update all desired values to Smart Dashboard */
  public abstract void updateSmartDashboard();

  /** Where you should set up the trackables to be logged */
  public abstract void setUpTrackables();
}
