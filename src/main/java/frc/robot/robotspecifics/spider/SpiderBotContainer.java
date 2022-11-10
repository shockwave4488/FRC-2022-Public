package frc.robot.robotspecifics.spider;

// import edu.wpi.first.wpilibj.SPI;
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.BaseRobotContainer;
import frc.lib.PreferencesParser;
import frc.lib.logging.Logger;
// import frc.lib.sensors.Limelight;
// import frc.lib.sensors.Limelight.DistanceEstimationConstants;
// import frc.lib.sensors.NavX;
// import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.c2022.Indexer;
import frc.robot.subsystems.c2022.Shooter;
// import org.json.simple.JSONObject;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class SpiderBotContainer extends BaseRobotContainer {
  // private final NavX m_gyro;
  private final Shooter shooter;
  private final Indexer indexer;
  // private final XboxController driverJoystick;
  // private final XboxController operatorJoystick;
  // private final Limelight shooterLimelight;
  // private final DistanceEstimationConstants shooterLimelightDistEstConstants;

  /*
  private DistanceEstimationConstants getShooterLimelightConstants() {
    JSONObject shooterLimelightDistEstConstantsJSON =
        prefs.getJSONObject("LimelightShooterConstants");
    return new DistanceEstimationConstants(
        ((Number) shooterLimelightDistEstConstantsJSON.get("camHeight")).doubleValue(),
        ((Number) shooterLimelightDistEstConstantsJSON.get("targetHeight")).doubleValue(),
        ((Number) shooterLimelightDistEstConstantsJSON.get("camToNormalAngle")).doubleValue());
  }
  */

  /**
   * The robot container for our spider bot, this is where all classes relevant to this robot are
   * created and where its default command(s) are set
   */
  public SpiderBotContainer(PreferencesParser prefs, Logger logger) {
    super(prefs, logger);

    // m_gyro = new NavX(SPI.Port.kMXP);

    // shooterLimelightDistEstConstants = getShooterLimelightConstants(); // 21, 103, 14
    /* The middle of the hub vision targets (above) are 103 inches off the ground, the other two values are just estimates.
    We also need to consider using an interpolation table instead of the above */

    /*
    shooterLimelight =
        new Limelight(
            prefs.getString("LimelightShooterName"), shooterLimelightDistEstConstants, logger);
    */

    shooter =
        new Shooter(
            prefs.getInt("ShooterMFlywheelID"),
            prefs.getInt("ShooterFFlywheelID"),
            prefs.getInt("Shooter_PCM_ID"),
            prefs.getInt("HoodServo1ID"),
            prefs.getInt("HoodServo2ID"),
            logger,
            prefs);

    indexer =
        new Indexer(
            prefs.getInt("ConveyorID"),
            prefs.getInt("EntranceBBID"),
            prefs.getInt("MiddleBBID"),
            prefs.getInt("FlywheelBBID"),
            logger,
            prefs);

    // driverJoystick = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
    // operatorJoystick = new XboxController(OIConstants.OPERATOR_CONTROLLER_PORT);

    addSubsystems();
    configureButtonBindings();
  }

  protected void addSubsystems() {
    subsystems.add(shooter);
    subsystems.add(indexer);
  }

  protected void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    return null;
  }
}
