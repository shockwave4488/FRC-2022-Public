package frc.robot.robotspecifics.swerve;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.BaseRobotContainer;
import frc.lib.PreferencesParser;
import frc.lib.drive.SwerveParameters;
import frc.lib.logging.Logger;
import frc.lib.operator.CircularDeadzone;
import frc.lib.operator.I2DDeadzoneCalculator;
import frc.lib.operator.IDeadzoneCalculator;
import frc.lib.operator.SquareDeadzoneCalculator;
import frc.lib.sensors.NavX;
import frc.lib.sensors.vision.Limelight;
import frc.lib.sensors.vision.VisionCamera.CameraPositionConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Robot;
import frc.robot.autonomous.modes.AutonomousChooser;
import frc.robot.autonomous.modes.AutonomousTrajectories;
import frc.robot.commands.drive.LockedSwerveDrive;
import frc.robot.commands.eruption.defaults.DefaultSwerveDrive;
import frc.robot.commands.eruption.drive.SwerveTurnToHub;
import frc.robot.commands.eruption.drive.VisionAlignToTarget;
import frc.robot.subsystems.drive.ISwerveModule;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.drive.SwerveModuleNeos;
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
  private final CommandXboxController driverJoystick;
  private final ISwerveModule m_frontLeft;
  private final ISwerveModule m_frontRight;
  private final ISwerveModule m_backLeft;
  private final ISwerveModule m_backRight;
  private final Limelight limelight;
  private final AutonomousChooser autonomousChooser;

  private SwerveParameters[] getSwerveParameters() {
    JSONObject swerveParametersJSONFL = prefs.getJSONObject("SwerveParametersFL");
    SwerveParameters swerveParametersFL = new SwerveParameters(swerveParametersJSONFL);
    JSONObject swerveParametersJSONFR = prefs.getJSONObject("SwerveParametersFR");
    SwerveParameters swerveParametersFR = new SwerveParameters(swerveParametersJSONFR);
    JSONObject swerveParametersJSONBL = prefs.getJSONObject("SwerveParametersBL");
    SwerveParameters swerveParametersBL = new SwerveParameters(swerveParametersJSONBL);
    JSONObject swerveParametersJSONBR = prefs.getJSONObject("SwerveParametersBR");
    SwerveParameters swerveParametersBR = new SwerveParameters(swerveParametersJSONBR);

    return new SwerveParameters[] {
      swerveParametersFL, swerveParametersFR, swerveParametersBL, swerveParametersBR
    };
  }

  private ISwerveModule[] getSwerveModules() {
    return new ISwerveModule[] {
      m_frontLeft, m_frontRight, m_backLeft, m_backRight,
    };
  }

  private CameraPositionConstants getLimelightConstants() {
    JSONObject limelightConstantsJSON = prefs.getJSONObject("LimelightShooterConstants");
    return new CameraPositionConstants(
        new Transform3d(
            new Translation3d(
                ((Number) limelightConstantsJSON.get("X")).doubleValue(),
                ((Number) limelightConstantsJSON.get("Y")).doubleValue(),
                ((Number) limelightConstantsJSON.get("Z")).doubleValue()),
            new Rotation3d(
                Math.toRadians(((Number) limelightConstantsJSON.get("Roll")).doubleValue()),
                Math.toRadians(((Number) limelightConstantsJSON.get("Pitch")).doubleValue()),
                Math.toRadians(((Number) limelightConstantsJSON.get("Yaw")).doubleValue()))));
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
    m_frontLeft = new SwerveModuleNeos(swerveParameters[0], logger, prefs);
    m_frontRight = new SwerveModuleNeos(swerveParameters[1], logger, prefs);
    m_backLeft = new SwerveModuleNeos(swerveParameters[2], logger, prefs);
    m_backRight = new SwerveModuleNeos(swerveParameters[3], logger, prefs);

    swerve = new SwerveDrive(m_gyro, getSwerveModules(), logger);
    driverJoystick = new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);
    limelight =
        new Limelight(
            prefs.getString("LimelightShooterName"),
            getLimelightConstants(),
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
            logger,
            prefs);

    swerve.setDefaultCommand(getNewDefaultSwerveDriveCommand());

    addSubsystems();
    configureButtonBindings();
  }

  protected void addSubsystems() {
    subsystems.add(swerve);
    subsystems.add(limelight);
  }

  protected void configureButtonBindings() {
    driverJoystick
        .x()
        .onTrue(
            new SwerveTurnToHub(
                swerve,
                m_gyro,
                DriveTrainConstants.SWERVE_DRIVE_MAX_SPEED,
                DriveTrainConstants.SWERVE_ROTATION_SPEED,
                () ->
                    circularDeadzone.deadzone(
                        driverJoystick.getLeftY() * -1, driverJoystick.getLeftX() * -1)));

    driverJoystick.rightTrigger(0.75).toggleOnTrue(new LockedSwerveDrive(swerve));

    driverJoystick
        .y()
        .toggleOnTrue(
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
        driverJoystick.start());
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getCommand();
  }
}
