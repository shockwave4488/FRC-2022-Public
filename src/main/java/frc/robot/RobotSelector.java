package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.IRobotContainer;
import frc.lib.PreferenceDoesNotExistException;
import frc.lib.PreferencesParser;
import frc.lib.logging.Logger;
import frc.robot.robotspecifics.bareroborio.BareRoboRIORobotContainer;
import frc.robot.robotspecifics.c2022.C2022RobotContainer;
import frc.robot.robotspecifics.candy.CandyRobotContainer;
import frc.robot.robotspecifics.practice.PracticeRobotContainer;
import frc.robot.robotspecifics.spider.SpiderBotContainer;
import frc.robot.robotspecifics.swerve.SwerveRobotContainer;

public class RobotSelector {
  private final PreferencesParser prefs;
  private final Logger logger;
  private IRobotContainer container;

  public RobotSelector(PreferencesParser prefs, Logger logger) {
    this.prefs = prefs;
    this.logger = logger;
  }

  public IRobotContainer getRobot() {
    String robotName;
    try {
      robotName = prefs.getString("RobotName");
      SmartDashboard.putString("Selected Robot:", robotName);
    } catch (PreferenceDoesNotExistException e) {
      container = new BareRoboRIORobotContainer(prefs, logger);
      SmartDashboard.putString("Selected Robot:", "BareRIO - RobotName key does not exist!");
      System.out.println("Selected Robot: BareRIO - RobotName key does not exist!");
      logger.writeRaw("Selected Robot: BareRIO - RobotName key does not exist!");

      return container;
    }

    if (robotName.equals("C2022")) {
      container = new C2022RobotContainer(prefs, logger);
    } else if (robotName.equals("P2022")) {
      container = new PracticeRobotContainer(prefs, logger);
    } else if (robotName.equals("Candy")) {
      container = new CandyRobotContainer(prefs, logger);
    } else if (robotName.equals("Spider")) {
      container = new SpiderBotContainer(prefs, logger);
    } else if (robotName.equals("Swerve")) {
      container = new SwerveRobotContainer(prefs, logger);
    } else if (robotName.equals("BareRoboRIO")) {
      container = new BareRoboRIORobotContainer(prefs, logger);
    } else {
      container = new BareRoboRIORobotContainer(prefs, logger);
      SmartDashboard.putString("Selected Robot:", "BareRIO - Invalid RobotName value in prefs!");
      System.out.println("Selected Robot: BareRIO - Invalid RobotName value in prefs!");
      logger.writeRaw("Selected Robot: BareRIO - Invalid RobotName value in prefs!");
    }

    // TODO: Add more of our robot types and make their containers
    return container;
  }
}
