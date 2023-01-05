package frc.robot.robotspecifics.practice;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
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
import frc.lib.sensors.NavX;
import frc.lib.sensors.vision.Limelight;
import frc.lib.sensors.vision.VisionCamera.CameraPositionConstants;
import frc.lib.util.app.Util;
import frc.lib.util.app.math.JSONPosition;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Robot;
import frc.robot.autonomous.modes.AutonomousChooser;
import frc.robot.autonomous.modes.AutonomousTrajectories;
import frc.robot.commands.c2022.defaults.DefaultSwerveDrive;
import frc.robot.commands.c2022.drive.LockedSwerveDrive;
import frc.robot.commands.c2022.drive.SwerveTurnToHUB;
import frc.robot.commands.c2022.drive.VisionAlignToTarget;
import frc.robot.subsystems.c2022.SmartPCM;
import frc.robot.subsystems.drive.ISwerveModule;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.drive.SwerveModuleFalcons;
import org.json.simple.JSONObject;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class PracticeRobotContainer extends BaseRobotContainer {
  private final NavX m_gyro;
  private final SwerveDrive swerve;
  private final SwerveParameters[] swerveParameters;
  private final I2DDeadzoneCalculator circularDeadzone;
  private final IDeadzoneCalculator squareDeadzone;
  private final XboxController driverJoystick;
  private final ISwerveModule m_frontLeft;
  private final ISwerveModule m_frontRight;
  private final ISwerveModule m_backLeft;
  private final ISwerveModule m_backRight;
  private final Limelight limelight;
  private final SmartPCM smartPCM;
  private final AutonomousChooser autonomousChooser;

  private SwerveParameters[] getSwerveParameters() {
    JSONObject swerveParametersJSONFL = prefs.getJSONObject("SwerveParametersFL");
    SwerveParameters swerveParametersFL =
        Util.toObj(swerveParametersJSONFL, SwerveParameters.class);
    JSONObject swerveParametersJSONFR = prefs.getJSONObject("SwerveParametersFR");
    SwerveParameters swerveParametersFR =
        Util.toObj(swerveParametersJSONFR, SwerveParameters.class);
    JSONObject swerveParametersJSONBL = prefs.getJSONObject("SwerveParametersBL");
    SwerveParameters swerveParametersBL =
        Util.toObj(swerveParametersJSONBL, SwerveParameters.class);
    JSONObject swerveParametersJSONBR = prefs.getJSONObject("SwerveParametersBR");
    SwerveParameters swerveParametersBR =
        Util.toObj(swerveParametersJSONBR, SwerveParameters.class);
    return new SwerveParameters[] {
      swerveParametersFL, swerveParametersFR, swerveParametersBL, swerveParametersBR
    };
  }

  private ISwerveModule[] getSwerveModules() {
    return new ISwerveModule[] {
      m_frontLeft, m_frontRight, m_backLeft, m_backRight,
    };
  }

  private CameraPositionConstants getCameraPositionConsts(JSONObject limelightConstantsJSON) {
    JSONObject limelightPositionJSON = (JSONObject) limelightConstantsJSON.get("Position");
    return new CameraPositionConstants(
        Util.toObj(limelightPositionJSON, JSONPosition.class).toTransform());
  }

  /**
   * The robot container for our basic swerve drive robot, this is where all classes relevant to
   * this robot are created and where its default command(s) are set
   */
  public PracticeRobotContainer(PreferencesParser prefs, Logger logger) {
    super(prefs, logger);

    m_gyro = new NavX(SPI.Port.kMXP);
    circularDeadzone = new CircularDeadzone(OIConstants.DEFAULT_CONTROLLER_DEADZONE);
    squareDeadzone = new SquareDeadzoneCalculator(OIConstants.DEFAULT_CONTROLLER_DEADZONE);
    swerveParameters = getSwerveParameters();
    m_frontLeft = new SwerveModuleFalcons(swerveParameters[0], logger, prefs);
    m_frontRight = new SwerveModuleFalcons(swerveParameters[1], logger, prefs);
    m_backLeft = new SwerveModuleFalcons(swerveParameters[2], logger, prefs);
    m_backRight = new SwerveModuleFalcons(swerveParameters[3], logger, prefs);

    smartPCM = new SmartPCM(prefs.getInt("PCM_ID"));

    swerve = new SwerveDrive(m_gyro, getSwerveModules(), logger);
    driverJoystick = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);

    JSONObject limelightPrefs = prefs.getJSONObject("LimelightConstants");
    limelight =
        new Limelight(
            (String) limelightPrefs.get("Name"), getCameraPositionConsts(limelightPrefs), logger);

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
            logger,
            prefs);

    swerve.setDefaultCommand(getNewDefaultSwerveDriveCommand());
    smartPCM.setDefaultCommand(
        new StartEndCommand(() -> smartPCM.startCompressor(), () -> {}, smartPCM)
            .withName("CompressorCommand"));

    addSubsystems();
    configureButtonBindings();
  }

  protected void addSubsystems() {
    subsystems.add(swerve);
    subsystems.add(limelight);
    subsystems.add(smartPCM);
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
