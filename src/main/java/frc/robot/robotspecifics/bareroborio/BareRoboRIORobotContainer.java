package frc.robot.robotspecifics.bareroborio;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.BaseRobotContainer;
import frc.lib.PreferencesParser;
import frc.lib.logging.Logger;

public class BareRoboRIORobotContainer extends BaseRobotContainer {

  public BareRoboRIORobotContainer(PreferencesParser prefs, Logger logger) {
    super(prefs, logger);
  }

  public Command getAutonomousCommand() {
    return null;
  }

  protected void configureButtonBindings() {}

  protected void addSubsystems() {}
}
