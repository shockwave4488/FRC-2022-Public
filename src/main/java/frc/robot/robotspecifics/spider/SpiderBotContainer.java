package frc.robot.robotspecifics.spider;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.lib.BaseRobotContainer;
import frc.lib.PreferencesParser;
import frc.lib.logging.Logger;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SmartPCM;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class SpiderBotContainer extends BaseRobotContainer {
  private final SmartPCM smartPCM;

  @SuppressWarnings("unused")
  private final Solenoid extensionSolenoid;

  @SuppressWarnings("unused")
  private final Solenoid retractionSolenoid;

  @SuppressWarnings("unused")
  private final XboxController driverJoystick;

  /**
   * The robot container for our spider bot, this is where all classes relevant to this robot are
   * created and where its default command(s) are set
   */
  public SpiderBotContainer(PreferencesParser prefs, Logger logger) {
    super(prefs, logger);

    smartPCM = new SmartPCM(prefs.getInt("PCM_ID"));

    smartPCM.setDefaultCommand(
        new StartEndCommand(() -> smartPCM.startCompressor(), () -> {}, smartPCM)
            .withName("CompressorCommand"));

    extensionSolenoid =
        new Solenoid(
            prefs.tryGetInt("PCM_ID", 0),
            PneumaticsModuleType.CTREPCM,
            prefs.tryGetInt("ClimberSolenoidExtendID", 1));
    retractionSolenoid =
        new Solenoid(
            prefs.tryGetInt("PCM_ID", 0),
            PneumaticsModuleType.CTREPCM,
            prefs.tryGetInt("ClimberSolenoidRetractID", 1));

    driverJoystick = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);

    addSubsystems();
    configureButtonBindings();
  }

  protected void addSubsystems() {
    subsystems.add(smartPCM);
  }

  protected void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    return null;
  }
}
