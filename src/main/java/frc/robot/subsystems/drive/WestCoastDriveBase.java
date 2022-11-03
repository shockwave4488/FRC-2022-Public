package frc.robot.subsystems.drive;

import frc.lib.wpiextensions.ShockwaveSubsystemBase;

public abstract class WestCoastDriveBase extends ShockwaveSubsystemBase {

  /**
   * The function that will drive a west coast robot
   *
   * @param foward The forward/backward speed of the robot
   * @param rot How fast the robot should turn
   */
  public abstract void driveWithJoysticks(double foward, double rot);
}
