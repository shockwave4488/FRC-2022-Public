package frc.lib;

import edu.wpi.first.wpilibj2.command.Command;

public interface IRobotContainer {
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand();

  /** Used to run the uponStart() function in each subsystem */
  public void runOnStart();

  /** Used to run the uponStop() function in each subsystem */
  public void runOnStop();

  /** Used to run the zeroSensors() function in each subsystem */
  public void runZeroSensors();

  /** Used to run the updateSmartDashboard function in each subsystem */
  public void runUpdateSmartDashboard();

  /** Used to run the setUpTrackables function in each subsystem */
  public void runSetUpTrackables();
}
