package frc.robot.robotspecifics.eruption;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.autonomous.modes.AutonomousChooser;
import frc.robot.autonomous.modes.AutonomousTrajectories;
import frc.robot.commands.drive.SwerveDriveWithHeading;
import frc.robot.commands.eruption.LEDs.RainbowLEDPattern;
import frc.robot.commands.eruption.climber.ClimbButtonBox;
import frc.robot.commands.eruption.climber.ClimberLiftToHeight;
import frc.robot.commands.eruption.defaults.DefaultClimber;
import frc.robot.commands.eruption.defaults.DefaultIndexerLoad;
import frc.robot.commands.eruption.defaults.DefaultIntakeRetracted;
import frc.robot.commands.eruption.defaults.DefaultShooter;
import frc.robot.commands.eruption.defaults.DefaultSwerveDrive;
import frc.robot.commands.eruption.drive.SwerveTurnToHub;
import frc.robot.commands.eruption.drive.TurnToHubPoseThenVision;
import frc.robot.commands.eruption.drive.VisionAlignToTarget;
import frc.robot.commands.eruption.intake.ColorIntake;
import frc.robot.commands.eruption.intake.ColorlessIntake;
import frc.robot.commands.eruption.intake.PurgeBack;
import frc.robot.commands.eruption.shooter.CalculatedShot;
import frc.robot.commands.eruption.shooter.LaunchFenderShot;
import frc.robot.commands.eruption.shooter.PurgeForward;
import frc.robot.commands.eruption.shooter.SpinFlywheel;
import frc.robot.subsystems.SmartPCM;
import frc.robot.subsystems.drive.ISwerveModule;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.drive.SwerveModuleFalcons;
import frc.robot.subsystems.eruption.Climber;
import frc.robot.subsystems.eruption.Indexer;
import frc.robot.subsystems.eruption.Intake;
import frc.robot.subsystems.eruption.LEDController;
import frc.robot.subsystems.eruption.Shooter;
import java.util.function.Supplier;
import org.json.simple.JSONObject;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class EruptionRobotContainer extends BaseRobotContainer {
  private final NavX m_gyro;
  private final SwerveDrive swerve;
  private final SwerveParameters[] swerveParameters;
  private final I2DDeadzoneCalculator circularDeadzone;
  private final I2DDeadzoneCalculator bigCircularDeadzone;
  private final IDeadzoneCalculator squareDeadzone;
  private final CommandXboxController driverJoystick;
  private final CommandGenericHID buttonBox;
  private final ISwerveModule m_frontLeft;
  private final ISwerveModule m_frontRight;
  private final ISwerveModule m_backLeft;
  private final ISwerveModule m_backRight;
  private final Limelight shooterLimelight;
  private final Shooter shooter;
  private final Climber climber;
  private final Indexer indexer;
  private final Indexer.StateSupplier indexerStates;
  private final Intake intake;
  private final LEDController ledController;
  private final SmartPCM smartPCM;
  private final UsbCamera usbCamera;
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

  private CameraPositionConstants getShooterLimelightConstants() {
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
  public EruptionRobotContainer(PreferencesParser prefs, Logger logger) {
    super(prefs, logger);

    m_gyro = new NavX(SPI.Port.kMXP);
    circularDeadzone = new CircularDeadzone(OIConstants.DEFAULT_CONTROLLER_DEADZONE);
    bigCircularDeadzone = new CircularDeadzone(OIConstants.BIG_CONTROLLER_DEADZONE);
    squareDeadzone = new SquareDeadzoneCalculator(OIConstants.DEFAULT_CONTROLLER_DEADZONE);
    swerveParameters = getSwerveParameters();
    m_frontLeft = new SwerveModuleFalcons(swerveParameters[0], logger, prefs);
    m_frontRight = new SwerveModuleFalcons(swerveParameters[1], logger, prefs);
    m_backLeft = new SwerveModuleFalcons(swerveParameters[2], logger, prefs);
    m_backRight = new SwerveModuleFalcons(swerveParameters[3], logger, prefs);

    shooterLimelight =
        new Limelight(
            prefs.getString("LimelightShooterName"), getShooterLimelightConstants(), logger);

    shooter =
        new Shooter(
            prefs.getInt("ShooterMFlywheelID"),
            prefs.getInt("ShooterFFlywheelID"),
            prefs.getInt("Shooter_PCM_ID"),
            prefs.getInt("HoodServo1ID"),
            prefs.getInt("HoodServo2ID"),
            logger,
            prefs);
    climber =
        new Climber(
            prefs.getInt("ClimberID"),
            logger,
            prefs,
            prefs.getInt("ClimberLeftInductiveID"),
            prefs.getInt("ClimberRightInductiveID"));
    indexer =
        new Indexer(
            prefs.getInt("ConveyorID"),
            prefs.getInt("EntranceBBID"),
            prefs.getInt("MiddleBBID"),
            prefs.getInt("FlywheelBBID"),
            logger,
            prefs);
    indexerStates = indexer.getIndexerStates();
    intake =
        new Intake(
            prefs.getInt("IntakeTopRollerID"),
            prefs.getInt("IntakeBottomRollerID"),
            prefs.getInt("Intake_PCM_ID"),
            prefs.getInt("IntakePistonID"),
            logger,
            prefs.getInt("IntakeColorSensorID"),
            prefs.getInt("IntakeBallSensorID"));
    smartPCM = new SmartPCM(prefs.getInt("Intake_PCM_ID"));
    ledController = new LEDController();

    swerve = new SwerveDrive(m_gyro, getSwerveModules(), logger);
    /*
    Set initial pose to have the robot pointing at the HUB so the robot doesn't snap upon
    enabling This will be overridden in real matches because autonomousInit will run in Robot.java.
    */
    swerve.resetOdometry(new Pose2d(0, 4.15, new Rotation2d(0)));
    driverJoystick = new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);
    buttonBox = new CommandGenericHID(OIConstants.BUTTON_BOX_PORT);

    // Backup default commands if we need to quickly switch what we're using
    // swerve.setDefaultCommand(getNewHeadingSwerveDriveCommand(false));
    // swerve.setDefaultCommand(getNewVisionAlignToTargetCommand(false));
    swerve.setDefaultCommand(
        new TurnToHubPoseThenVision(
            getNewSwerveTurnToHubCommand(), getNewVisionAlignToTargetCommand(false)));
    intake.setDefaultCommand(new DefaultIntakeRetracted(intake));
    smartPCM.setDefaultCommand(
        new StartEndCommand(() -> smartPCM.startCompressor(), () -> {}, smartPCM)
            .withName("CompressorCommand"));
    climber.setDefaultCommand(new DefaultClimber(climber));
    indexer.setDefaultCommand(new DefaultIndexerLoad(indexer));
    // shooter.setDefaultCommand(new ShooterWait(shooter));
    shooter.setDefaultCommand(
        new DefaultShooter(
            shooter, shooterLimelight, indexerStates::getFlywheelBeamBreak, swerve::getOdometry));
    usbCamera = CameraServer.startAutomaticCapture();
    usbCamera.setResolution(320, 180); // 480, 270
    usbCamera.setFPS(15); // 15

    autonomousChooser =
        new AutonomousChooser(
            new AutonomousTrajectories(swerve, logger),
            swerve,
            intake,
            shooter,
            indexer,
            smartPCM,
            m_gyro,
            shooterLimelight,
            logger,
            prefs);

    addSubsystems();
    configureButtonBindings();
  }

  protected void addSubsystems() {
    subsystems.add(swerve);
    subsystems.add(shooterLimelight);
    subsystems.add(shooter);
    subsystems.add(climber);
    subsystems.add(indexer);
    subsystems.add(intake);
    subsystems.add(ledController);
    subsystems.add(smartPCM);
  }

  protected void configureButtonBindings() {

    // Driver bindings
    driverJoystick.x().onTrue(getNewSwerveTurnToHubCommand());
    driverJoystick
        .back()
        .toggleOnTrue(
            new DefaultSwerveDrive(
                    swerve,
                    DriveTrainConstants.SWERVE_DRIVE_MAX_SPEED,
                    DriveTrainConstants.SWERVE_ROTATION_SPEED * -1,
                    () ->
                        circularDeadzone.deadzone(
                            driverJoystick.getLeftY() * -1, driverJoystick.getLeftX() * -1),
                    () -> squareDeadzone.deadzone(driverJoystick.getRightX()),
                    driverJoystick.start())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    driverJoystick.leftTrigger(0.5).whileTrue(new PurgeBack(intake, indexer, shooter));
    driverJoystick.rightTrigger(0.5).toggleOnTrue(new ColorlessIntake(intake));
    driverJoystick
        .rightBumper()
        .toggleOnTrue(
            getNewCalculatedShotCommand()
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    driverJoystick.leftBumper().whileTrue(new PurgeForward(indexer, shooter));

    // Activated if driver joystick value is outside of deadzone
    Trigger driverRightJoystick =
        new Trigger(
            () ->
                bigCircularDeadzone.isPastDeadzone(
                    driverJoystick.getRightY() * -1, driverJoystick.getRightX() * -1));
    driverRightJoystick.whileTrue(new RepeatCommand(getNewHeadingSwerveDriveCommand(false)));

    driverJoystick
        .povUp()
        .onTrue(new InstantCommand(() -> shooter.setRPMOffset(shooter.getRPMOffset() + 10)));
    driverJoystick
        .povDown()
        .onTrue(new InstantCommand(() -> shooter.setRPMOffset(shooter.getRPMOffset() - 10)));

    // Button box/operator bindings
    buttonBox.button(1).toggleOnTrue(getNewVisionAlignToTargetCommand(false));
    // Toggle so operator can stop if aligned to the wrong target
    buttonBox
        .button(3)
        .toggleOnTrue(
            getNewCalculatedShotCommand()
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    buttonBox.button(4).onTrue(new InstantCommand(() -> shooterLimelight.takeSnapshot()));
    buttonBox
        .button(5)
        .onTrue(
            new LaunchFenderShot(
                shooter,
                indexer,
                smartPCM,
                ShooterConstants.FENDER_RPM,
                ShooterConstants.FENDER_HOOD_INPUT));
    Command chargeShooter =
        new SpinFlywheel(
            shooter,
            shooterLimelight,
            indexerStates::getFlywheelBeamBreak,
            swerve::getOdometry,
            true,
            true,
            false);
    buttonBox.button(6).whileTrue(chargeShooter);
    Trigger alignedTrigger =
        new Trigger(() -> climber.getLeftInductiveSensor() && climber.getRightInductiveSensor());
    alignedTrigger.onTrue(new RainbowLEDPattern(ledController));
    buttonBox
        .button(7)
        .onTrue(new InstantCommand(() -> shooter.setRPMOffset(shooter.getRPMOffset() - 10)));
    buttonBox
        .button(8)
        .onTrue(new InstantCommand(() -> shooter.setRPMOffset(shooter.getRPMOffset() + 10)));
    buttonBox
        .button(9)
        .onTrue(
            new ClimberLiftToHeight(climber, () -> buttonBox.getHID().getRawButton(13))
                .andThen(
                    new ClimbButtonBox(
                        climber,
                        shooter,
                        () -> buttonBox.getHID().getRawButton(9),
                        () -> buttonBox.getHID().getRawButton(13))));
    buttonBox.button(10).whileTrue(new PurgeBack(intake, indexer, shooter));
    buttonBox.button(11).whileTrue(new PurgeForward(indexer, shooter));
    buttonBox.button(14).whileTrue(new ColorIntake(intake, indexerStates, false));
    buttonBox.button(15).whileTrue(new ColorlessIntake(intake));
    buttonBox.button(16).whileTrue(new ColorIntake(intake, indexerStates, true));

    SmartDashboard.putData(
        "Limelight Snapshot",
        new InstantCommand(() -> shooterLimelight.takeSnapshot()).withName("Take snapshot"));
  }

  private Supplier<double[]> getDriverLeftStickInput() {
    return () ->
        circularDeadzone.deadzone(driverJoystick.getLeftY() * -1, driverJoystick.getLeftX() * -1);
  }

  private Supplier<double[]> getDriverRightStickInput() {
    return () ->
        bigCircularDeadzone.deadzone(
            driverJoystick.getRightY() * -1, driverJoystick.getRightX() * -1);
  }

  private SwerveDriveWithHeading getNewHeadingSwerveDriveCommand(boolean targetTakeover) {
    /*
    The multiplication of SWERVE_ROTATION_SPEED by -1 and the first two getY/getX values being switched and multiplied by -1 are intentional.
    Multiplying SWERVE_ROTATION_SPEED by -1 corrects the direction of rotation of our robot, and we switch getY/getX and multiply them by -1 because the controller input is
    90 degrees off compared to the values WPILib utilities expect (particularly ChassisSpeeds)
    */

    return new SwerveDriveWithHeading(
        swerve,
        m_gyro,
        DriveTrainConstants.SWERVE_DRIVE_MAX_SPEED,
        DriveTrainConstants.SWERVE_ROTATION_SPEED * -1,
        getDriverLeftStickInput(),
        getDriverRightStickInput(),
        driverJoystick.start(),
        shooterLimelight,
        targetTakeover);
  }

  private VisionAlignToTarget getNewVisionAlignToTargetCommand(boolean stop) {
    return new VisionAlignToTarget(
        swerve,
        shooterLimelight,
        m_gyro,
        Constants.DriveTrainConstants.SWERVE_DRIVE_MAX_SPEED,
        Constants.DriveTrainConstants.SWERVE_ROTATION_SPEED,
        getDriverLeftStickInput(),
        10,
        stop);
  }

  private SwerveTurnToHub getNewSwerveTurnToHubCommand() {
    return new SwerveTurnToHub(
        swerve,
        m_gyro,
        DriveTrainConstants.SWERVE_DRIVE_MAX_SPEED,
        DriveTrainConstants.SWERVE_ROTATION_SPEED,
        () ->
            circularDeadzone.deadzone(
                driverJoystick.getLeftY() * -1, driverJoystick.getLeftX() * -1));
  }

  private CalculatedShot getNewCalculatedShotCommand() {
    return new CalculatedShot(
        shooter,
        indexer,
        smartPCM,
        swerve,
        m_gyro,
        getDriverLeftStickInput(),
        DriveTrainConstants.SWERVE_DRIVE_MAX_SPEED,
        DriveTrainConstants.SWERVE_ROTATION_SPEED,
        shooterLimelight,
        getNewSwerveTurnToHubCommand(),
        getNewHeadingSwerveDriveCommand(true));
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getCommand();
  }
}
