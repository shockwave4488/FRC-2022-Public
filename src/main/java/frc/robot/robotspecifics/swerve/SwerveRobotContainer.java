package frc.robot.robotspecifics.swerve;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.BaseRobotContainer;
import frc.lib.PreferencesParser;
import frc.lib.drive.SwerveParameters;
import frc.lib.logging.Logger;
import frc.lib.operator.AxisTrigger;
import frc.lib.operator.CircularDeadzone;
import frc.lib.operator.I2DDeadzoneCalculator;
import frc.lib.operator.IDeadzoneCalculator;
import frc.lib.operator.SquareDeadzoneCalculator;
import frc.lib.sensors.Limelight;
import frc.lib.sensors.Limelight.DistanceEstimationConstants;
import frc.lib.sensors.NavX;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.autonomous.modes.AutonomousChooser;
import frc.robot.autonomous.modes.AutonomousTrajectories;
import frc.robot.commands.c2022.defaults.DefaultSwerveDrive;
import frc.robot.commands.c2022.drive.LockedSwerveDrive;
import frc.robot.commands.c2022.drive.SwerveTurnToHUB;
import frc.robot.commands.c2022.drive.VisionAlignToTarget;
import frc.robot.subsystems.drive.ISwerveModule;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.drive.SwerveModuleNeos;
import java.util.ArrayList;
import org.json.simple.JSONObject;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class SwerveRobotContainer extends BaseRobotContainer {
  private final NavX m_gyro;
  private final SwerveDrive swerve;
  private final SwerveParameters[] swerveParameters;
  private final I2DDeadzoneCalculator circularDeadzone;
  private final IDeadzoneCalculator squareDeadzone;
  private final XboxController driverJoystick;
  private final ISwerveModule[] SModules;
  private final DistanceEstimationConstants limelightDistEstConstants;
  private final Limelight limelight;
  private AutonomousChooser autonomousChooser;

  private SwerveParameters[] getSwerveParameters() {
    JSONObject swerveParametersJSON = prefs.getJSONObject("SwerveParameters");
    ArrayList<SwerveParameters> swerveParamList = new ArrayList<SwerveParameters>();
    for (Object moduleParameters : swerveParametersJSON.values()) {
      swerveParamList.add(new SwerveParameters((JSONObject) moduleParameters));
    }
    return swerveParamList.toArray(SwerveParameters[]::new);
  }

  private DistanceEstimationConstants getLimelightConstants() {
    JSONObject shooterLimelightDistEstConstantsJSON =
        prefs.getJSONObject("LimelightShooterConstants");
    return new DistanceEstimationConstants(
        ((Number) shooterLimelightDistEstConstantsJSON.get("camHeight")).doubleValue(),
        ((Number) shooterLimelightDistEstConstantsJSON.get("targetHeight")).doubleValue(),
        ((Number) shooterLimelightDistEstConstantsJSON.get("camToNormalAngle")).doubleValue());
  }

  /**
   * The robot container for our basic swerve drive robot, this is where all classes relevant to
   * this robot are created and where its default command(s) are set
   */
  public SwerveRobotContainer(PreferencesParser prefs, Logger logger) {
    super(prefs, logger);

    m_gyro = new NavX(SPI.Port.kMXP);
    circularDeadzone = new CircularDeadzone(OIConstants.DEFAULT_CONTROLLER_DEADZONE);
    squareDeadzone = new SquareDeadzoneCalculator(OIConstants.DEFAULT_CONTROLLER_DEADZONE);
    swerveParameters = getSwerveParameters();
    SModules = new ISwerveModule[swerveParameters.length];

    for (int i = 0; i < SModules.length; i++) {
      SModules[i] = new SwerveModuleNeos(swerveParameters[i], logger, prefs);
    }

    swerve = new SwerveDrive(m_gyro, SModules, logger);
    driverJoystick = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
    limelightDistEstConstants = getLimelightConstants();
    limelight =
        new Limelight(
            prefs.getString("LimelightShooterName"),
            limelightDistEstConstants,
            logger); // UPDATE VALUES HERE
    autonomousChooser =
        new AutonomousChooser(
            new AutonomousTrajectories(swerve, logger),
            swerve,
            null,
            null,
            null,
            null,
            m_gyro,
            limelight,
            logger);

    swerve.setDefaultCommand(getNewDefaultSwerveDriveCommand());

    addSubsystems();
    configureButtonBindings();
  }

  protected void addSubsystems() {
    subsystems.add(swerve);
    subsystems.add(limelight);
  }

  protected void configureButtonBindings() {
    JoystickButton driverX = new JoystickButton(driverJoystick, XboxController.Button.kX.value);
    driverX.whenPressed(
        new SwerveTurnToHUB(
            swerve,
            limelight,
            m_gyro,
            DriveTrainConstants.SWERVE_DRIVE_MAX_SPEED,
            DriveTrainConstants.SWERVE_ROTATION_SPEED,
            () ->
                circularDeadzone.deadzone(
                    driverJoystick.getLeftY() * -1, driverJoystick.getLeftX() * -1)));

    AxisTrigger driverRightTrigger =
        new AxisTrigger(driverJoystick, XboxController.Axis.kRightTrigger.value, 0.75);
    driverRightTrigger.toggleWhenActive(new LockedSwerveDrive(swerve));

    JoystickButton driverY = new JoystickButton(driverJoystick, XboxController.Button.kY.value);
    driverY.toggleWhenPressed(
        new VisionAlignToTarget(
            swerve,
            limelight,
            m_gyro,
            DriveTrainConstants.SWERVE_DRIVE_MAX_SPEED,
            DriveTrainConstants.SWERVE_ROTATION_SPEED,
            () ->
                circularDeadzone.deadzone(
                    driverJoystick.getLeftY() * -1, driverJoystick.getLeftX() * -1),
            10,
            true));
  }

  private DefaultSwerveDrive getNewDefaultSwerveDriveCommand() {
    /*
    The multiplication of SWERVE_ROTATION_SPEED by -1 and the first two getY/getX values being switched and multiplied by -1 are intentional.
    Multiplying SWERVE_ROTATION_SPEED by -1 corrects the direction of rotation of our robot, and we switch getY/getX and multiply them by -1 because the controller input is
    90 degrees off compared to the values WPILib utilities expect (particularly ChassisSpeeds)
    */

    return new DefaultSwerveDrive(
        swerve,
        DriveTrainConstants.SWERVE_DRIVE_MAX_SPEED,
        DriveTrainConstants.SWERVE_ROTATION_SPEED * -1,
        () ->
            circularDeadzone.deadzone(
                driverJoystick.getLeftY() * -1, driverJoystick.getLeftX() * -1),
        () -> squareDeadzone.deadzone(driverJoystick.getRightX()),
        () -> driverJoystick.getStartButtonPressed());
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getCommand();
  }
}
