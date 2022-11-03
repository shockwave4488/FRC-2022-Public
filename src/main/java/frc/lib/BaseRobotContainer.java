package frc.lib;

import frc.lib.logging.Logger;
import frc.lib.wpiextensions.ShockwaveSubsystemBase;
import java.util.ArrayList;

public abstract class BaseRobotContainer implements IRobotContainer {
  protected ArrayList<ShockwaveSubsystemBase> subsystems;

  /* PreferencesParser & Logger objects are present in this class to remind people to pass them into
  RobotContainer constructors (which is why they aren't used here and the SpotBugs bypass below
  is used) */

  @edu.umd.cs.findbugs.annotations.SuppressFBWarnings(
      value = "URF_UNREAD_PUBLIC_OR_PROTECTED_FIELD",
      justification = "Used in dependent classes")
  protected PreferencesParser prefs;

  @edu.umd.cs.findbugs.annotations.SuppressFBWarnings(
      value = "URF_UNREAD_PUBLIC_OR_PROTECTED_FIELD",
      justification = "Used in dependent classes")
  protected Logger logger;

  protected BaseRobotContainer(PreferencesParser prefs, Logger logger) {
    subsystems = new ArrayList<>();
    this.prefs = prefs;
    this.logger = logger;
  }

  @Override
  public void runOnStart() {
    subsystems.forEach((s) -> s.onStart());
  }

  @Override
  public void runOnStop() {
    subsystems.forEach((s) -> s.onStop());
  }

  @Override
  public void runZeroSensors() {
    subsystems.forEach((s) -> s.zeroSensors());
  }

  @Override
  public void runUpdateSmartDashboard() {
    subsystems.forEach((s) -> s.updateSmartDashboard());
  }

  @Override
  public void runSetUpTrackables() {
    subsystems.forEach((s) -> s.setUpTrackables());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  protected abstract void configureButtonBindings();

  /**
   * Add each subsystem (each class that extends ShockwaveSubsystemBase) to the ArrayList created in
   * BaseRobotContainer
   */
  protected abstract void addSubsystems();
}
