package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** This class creates a template for which all swerve code should follow. */
public interface ISwerveModule {

  /** Gets the state the module is in. */
  SwerveModuleState getState();

  /** Gets the position of the module. */
  SwerveModulePosition getPosition();

  /**
   * Sets which state the module should be in.
   *
   * @param desiredState State the module should be in. May change depending on where in the code it
   *     is used.
   */
  void setDesiredState(SwerveModuleState desiredState);

  /** Gets which angle the module should be facing */
  double getDesiredAngle();

  /** Gets which speed the module should be going in meters/sec */
  double getDesiredSpeed();

  /**
   * @return The speed of the module's wheel in meters/sec
   */
  double getSpeed();

  /**
   * @return The angle of the module in degrees within the range (-180, 180]
   */
  double getAbsoluteAngleDegrees();

  /**
   * Updates values on SmartDashboard. If these aren't showing up, double check that the call to
   * this method in SwerveDrive.java is uncommented.
   */
  default void updateSmartDashboard() {}

  /** Call this method to run code upon enabling the robot */
  default void onStart() {}
}
