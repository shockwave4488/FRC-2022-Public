package frc.robot.robotspecifics.c2022;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.BaseRobotContainer;
import frc.lib.PreferencesParser;
import frc.lib.drive.SwerveParameters;
import frc.lib.logging.Logger;
import frc.lib.operator.AxisTrigger;
import frc.lib.operator.ButtonBox;
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
import frc.robot.commands.c2022.LEDs.RainbowLEDPattern;
import frc.robot.commands.c2022.climber.ClimbButtonBox;
import frc.robot.commands.c2022.climber.ClimberLiftToHeight;
import frc.robot.commands.c2022.defaults.DefaultClimber;
import frc.robot.commands.c2022.defaults.DefaultIndexerLoad;
import frc.robot.commands.c2022.defaults.DefaultIntakeRetracted;
import frc.robot.commands.c2022.defaults.DefaultShooter;
import frc.robot.commands.c2022.defaults.DefaultSwerveDrive;
import frc.robot.commands.c2022.drive.SwerveDriveWithHeading;
import frc.robot.commands.c2022.drive.SwerveTurnToHUB;
import frc.robot.commands.c2022.drive.TurnToHUBPoseThenVision;
import frc.robot.commands.c2022.drive.VisionAlignToTarget;
import frc.robot.commands.c2022.intake.ColorIntake;
import frc.robot.commands.c2022.intake.ColorlessIntake;
import frc.robot.commands.c2022.intake.PurgeBack;
import frc.robot.commands.c2022.shooter.CalculatedShot;
import frc.robot.commands.c2022.shooter.LaunchFenderShot;
import frc.robot.commands.c2022.shooter.PurgeForward;
import frc.robot.commands.c2022.shooter.SpinFlywheel;
import frc.robot.subsystems.c2022.Climber;
import frc.robot.subsystems.c2022.Indexer;
import frc.robot.subsystems.c2022.Intake;
import frc.robot.subsystems.c2022.LEDController;
import frc.robot.subsystems.c2022.Shooter;
import frc.robot.subsystems.c2022.SmartPCM;
import frc.robot.subsystems.drive.ISwerveModule;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.drive.SwerveModuleFalcons;
import java.util.function.Supplier;
import org.json.simple.JSONObject;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class C2022RobotContainer extends BaseRobotContainer {
  private final NavX m_gyro;
  private final SwerveDrive swerve;
  private final SwerveParameters[] swerveParameters;
  private final I2DDeadzoneCalculator circularDeadzone;
  private final I2DDeadzoneCalculator bigCircularDeadzone;
  private final IDeadzoneCalculator squareDeadzone;
  private final XboxController driverJoystick;
  private final ButtonBox buttonBox;
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
  private AutonomousChooser autonomousChooser;

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
  public C2022RobotContainer(PreferencesParser prefs, Logger logger) {
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
    driverJoystick = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
    buttonBox = new ButtonBox(OIConstants.BUTTON_BOX_PORT);

    // Backup default commands if we need to quickly switch what we're using
    // swerve.setDefaultCommand(getNewHeadingSwerveDriveCommand(false));
    // swerve.setDefaultCommand(getNewVisionAlignToTargetCommand(false));
    swerve.setDefaultCommand(
        new TurnToHUBPoseThenVision(
            getNewSwerveTurnToHUBCommand(), getNewVisionAlignToTargetCommand(false), swerve));
    intake.setDefaultCommand(new DefaultIntakeRetracted(intake));
    climber.setDefaultCommand(new DefaultClimber(climber));
    indexer.setDefaultCommand(new DefaultIndexerLoad(indexer));
    // shooter.setDefaultCommand(new ShooterWait(shooter));
    shooter.setDefaultCommand(
        new DefaultShooter(shooter, shooterLimelight, indexer, () -> swerve.getOdometry()));
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
    JoystickButton driverX = new JoystickButton(driverJoystick, XboxController.Button.kX.value);
    driverX.whenPressed(getNewSwerveTurnToHUBCommand());
    JoystickButton driverBack =
        new JoystickButton(driverJoystick, XboxController.Button.kBack.value);
    driverBack.toggleWhenPressed(
        new DefaultSwerveDrive(
            swerve,
            DriveTrainConstants.SWERVE_DRIVE_MAX_SPEED,
            DriveTrainConstants.SWERVE_ROTATION_SPEED * -1,
            () ->
                circularDeadzone.deadzone(
                    driverJoystick.getLeftY() * -1, driverJoystick.getLeftX() * -1),
            () -> squareDeadzone.deadzone(driverJoystick.getRightX()),
            () -> driverJoystick.getStartButtonPressed()),
        false);
    AxisTrigger driverLeftTrigger =
        new AxisTrigger(driverJoystick, XboxController.Axis.kLeftTrigger.value, 0.5);
    driverLeftTrigger.whileActiveContinuous(new PurgeBack(intake, indexer, shooter));
    AxisTrigger driverRightTrigger =
        new AxisTrigger(driverJoystick, XboxController.Axis.kRightTrigger.value, 0.5);
    driverRightTrigger.toggleWhenActive(new ColorlessIntake(intake));
    JoystickButton driverRightBumperButton =
        new JoystickButton(driverJoystick, XboxController.Button.kRightBumper.value);
    driverRightBumperButton.toggleWhenActive(getNewCalculatedShotCommand(), false);
    JoystickButton driverLeftBumperButton =
        new JoystickButton(driverJoystick, XboxController.Button.kLeftBumper.value);
    driverLeftBumperButton.whileActiveOnce(new PurgeForward(indexer, shooter));

    // Activated if driver joystick value is outside of deadzone
    Trigger driverRightJoystick =
        new Trigger(
            () ->
                bigCircularDeadzone.isPastDeadzone(
                    driverJoystick.getRightY() * -1, driverJoystick.getRightX() * -1));
    driverRightJoystick.whileActiveContinuous(getNewHeadingSwerveDriveCommand(false));

    Trigger dPadUp = new Trigger(() -> driverJoystick.getPOV() == 0);
    Trigger dPadDown = new Trigger(() -> driverJoystick.getPOV() == 180);
    dPadUp.whenActive(new InstantCommand(() -> shooter.setRPMOffset(shooter.getRPMOffset() + 10)));
    dPadDown.whenActive(
        new InstantCommand(() -> shooter.setRPMOffset(shooter.getRPMOffset() - 10)));

    // Button box
    Trigger bBoxButton1 = new Trigger(() -> buttonBox.button1()); // Vision align
    // Trigger bBoxButton2 = new Trigger(() -> buttonBox.button2());
    Trigger bBoxButton3 = new Trigger(() -> buttonBox.button3()); // Auto Shot
    Trigger bBoxButton4 = new Trigger(() -> buttonBox.button4());
    Trigger bBoxButton5 = new Trigger(() -> buttonBox.button5()); // Fender Shot
    Trigger bBoxButton6 = new Trigger(() -> buttonBox.button6()); // Charge Shooter
    Trigger bBoxButton7 = new Trigger(() -> buttonBox.button7()); // RPM Offset Down 10
    Trigger bBoxButton8 = new Trigger(() -> buttonBox.button8()); // RPM Offset Up 10
    Trigger bBoxButton9 = new Trigger(() -> buttonBox.button9()); // Arm up
    Trigger bBoxButton10 = new Trigger(() -> buttonBox.button10()); // Purge Back
    Trigger bBoxButton11 = new Trigger(() -> buttonBox.button11()); // Purge Forward
    // Trigger bBoxButton12 = new Trigger(() -> buttonBox.button12());
    Trigger bBoxButton13 = new Trigger(() -> buttonBox.button13()); // Arm down
    Trigger bBoxButton14 = new Trigger(() -> buttonBox.button14()); // Color intake
    Trigger bBoxButton15 = new Trigger(() -> buttonBox.button15()); // Colorless Intake
    Trigger bBoxButton16 = new Trigger(() -> buttonBox.button16()); // Hard Color Intake

    bBoxButton1.toggleWhenActive(getNewVisionAlignToTargetCommand(false));
    // Toggle so operator can stop if aligned to the wrong target
    bBoxButton3.toggleWhenActive(getNewCalculatedShotCommand(), false);
    bBoxButton4.whenActive(new InstantCommand(() -> shooterLimelight.takeSnapshot()));
    bBoxButton5.whenActive(
        new LaunchFenderShot(
            shooter,
            indexer,
            smartPCM,
            ShooterConstants.FENDER_RPM,
            ShooterConstants.FENDER_HOOD_INPUT));
    Command chargeShooter =
        new SpinFlywheel(
            shooter, shooterLimelight, indexer, () -> swerve.getOdometry(), true, true, false);
    bBoxButton6.whileActiveOnce(chargeShooter);
    Trigger alignedTrigger =
        new Trigger(() -> climber.getLeftInductiveSensor() && climber.getRightInductiveSensor());
    alignedTrigger.whenActive(new RainbowLEDPattern(ledController));
    bBoxButton7.whenActive(
        new InstantCommand(() -> shooter.setRPMOffset(shooter.getRPMOffset() - 10)));
    bBoxButton8.whenActive(
        new InstantCommand(() -> shooter.setRPMOffset(shooter.getRPMOffset() + 10)));
    bBoxButton9.whenActive(
        new ClimberLiftToHeight(climber, bBoxButton13)
            .andThen(new ClimbButtonBox(climber, shooter, bBoxButton9, bBoxButton13)));
    bBoxButton10.whileActiveOnce(new PurgeBack(intake, indexer, shooter));
    bBoxButton11.whileActiveOnce(new PurgeForward(indexer, shooter));
    bBoxButton14.whileActiveOnce(new ColorIntake(intake, indexerStates, false));
    bBoxButton15.whileActiveOnce(new ColorlessIntake(intake));
    bBoxButton16.whileActiveOnce(new ColorIntake(intake, indexerStates, true));

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
        () -> driverJoystick.getStartButtonPressed(),
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

  private SwerveTurnToHUB getNewSwerveTurnToHUBCommand() {
    return new SwerveTurnToHUB(
        swerve,
        shooterLimelight,
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
        getNewSwerveTurnToHUBCommand(),
        getNewHeadingSwerveDriveCommand(true));
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getCommand();
  }
}
