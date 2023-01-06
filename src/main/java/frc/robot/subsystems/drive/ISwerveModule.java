package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.SwerveModuleState;

/** This class creates a template for which all swerve code should follow. */
public interface ISwerveModule {

  /** Gets the state the module is in. */
  public SwerveModuleState getState();

  /**
   * Sets which state the module should be in.
   *
   * @param desiredState State the module should be in. May change depending on where in the code it
   *     is used.
   */
  public void setDesiredState(SwerveModuleState desiredState);

  /** Gets which angle the module should be facing */
  public double getDesiredAngle();

  /** Gets which speed the module should be going in meters/sec */
  public double getDesiredSpeed();

  /**
   * @return The speed of the module's wheel in meters/sec
   */
  public double getSpeed();

  /**
   * @return The angle of the module's wheels in degrees
   */
  public double getAbsoluteAngleDegrees();

  /** Updates the Smart Dashboard values */
  public void updateSmartDashboard();

  /** Call this method to prevent the module from moving from its current position */
  public void halt();

  /** Call this method to run code upon enabling the robot */
  public void onStart();
}
