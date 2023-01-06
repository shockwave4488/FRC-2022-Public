package frc.robot.robotspecifics.candy;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
import frc.lib.sensors.vision.PhotonCameras.AprilTagPhotonCamera;
import frc.lib.sensors.vision.VisionCamera.CameraPositionConstants;
import frc.lib.sensors.vision.VisionTargets.AprilTagPhotonTarget;
import frc.lib.sensors.vision.VisionTargets.AprilTagTarget;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.MockBuildWeek2023Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.Robot;
import frc.robot.autonomous.modes.AutonomousChooser.AutoPIDControllerContainer;
import frc.robot.commands.c2022.defaults.DefaultSwerveDrive;
import frc.robot.commands.c2022.drive.DriveAndFaceAprilTag;
import frc.robot.commands.c2022.drive.SwerveDriveToPosition;
import frc.robot.commands.mock2023.defaults.DefaultElevator;
import frc.robot.commands.mock2023.elevator.DropCandy;
import frc.robot.commands.mock2023.elevator.ElevatorDown;
import frc.robot.commands.mock2023.elevator.ElevatorUp;
import frc.robot.commands.mock2023.elevator.SnatchCandy;
import frc.robot.subsystems.c2022.SmartPCM;
import frc.robot.subsystems.drive.ISwerveModule;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.drive.SwerveModuleFalcons;
import frc.robot.subsystems.mock2023.Elevator;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import org.json.simple.JSONObject;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class CandyRobotContainer extends BaseRobotContainer {
  private final AutoPIDControllerContainer autoPidControllers;
  private final NavX m_gyro;
  private final SwerveDrive swerve;
  private final Elevator elevator;
  private final I2DDeadzoneCalculator circularDeadzone;
  private final IDeadzoneCalculator squareDeadzone;
  private final XboxController driverJoystick;
  private final XboxController operatorJoystick;
  private final ISwerveModule m_frontLeft;
  private final ISwerveModule m_frontRight;
  private final ISwerveModule m_backLeft;
  private final ISwerveModule m_backRight;
  private final AprilTagPhotonCamera camera;
  private final SmartPCM smartPCM;
  private int grabbedCubeId = -1;

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

  private CameraPositionConstants getCameraPositionConsts(JSONObject limelightConstantsJSON) {
    JSONObject limelightPositionJSON = (JSONObject) limelightConstantsJSON.get("Position");
    return new CameraPositionConstants(
        new Transform3d(
            new Translation3d(
                ((Number) limelightPositionJSON.get("X")).doubleValue(),
                ((Number) limelightPositionJSON.get("Y")).doubleValue(),
                ((Number) limelightPositionJSON.get("Z")).doubleValue()),
            new Rotation3d(
                Math.toRadians(((Number) limelightPositionJSON.get("Roll")).doubleValue()),
                Math.toRadians(((Number) limelightPositionJSON.get("Pitch")).doubleValue()),
                Math.toRadians(((Number) limelightPositionJSON.get("Yaw")).doubleValue()))));
  }

  /**
   * The robot container for our practice robot, this is where all classes relevant to this robot
   * are created and where its default command(s) are set
   */
  public CandyRobotContainer(PreferencesParser prefs, Logger logger) {
    super(prefs, logger);

    m_gyro = new NavX(SPI.Port.kMXP);
    circularDeadzone = new CircularDeadzone(OIConstants.DEFAULT_CONTROLLER_DEADZONE);
    squareDeadzone = new SquareDeadzoneCalculator(OIConstants.DEFAULT_CONTROLLER_DEADZONE);
    SwerveParameters[] swerveParameters = getSwerveParameters();
    m_frontLeft = new SwerveModuleFalcons(swerveParameters[0], logger, prefs);
    m_frontRight = new SwerveModuleFalcons(swerveParameters[1], logger, prefs);
    m_backLeft = new SwerveModuleFalcons(swerveParameters[2], logger, prefs);
    m_backRight = new SwerveModuleFalcons(swerveParameters[3], logger, prefs);

    JSONObject limelightPrefs = prefs.getJSONObject("LimelightConstants");
    camera =
        new AprilTagPhotonCamera(
            (String) limelightPrefs.get("Name"), getCameraPositionConsts(limelightPrefs), logger);

    smartPCM = new SmartPCM(prefs.getInt("PCM_ID"));

    elevator =
        new Elevator(
            prefs.getInt("HallEffectID"),
            prefs.getInt("CandyGrabberID"),
            prefs.getInt("ElevatorID"),
            prefs.getInt("PCM_ID"));

    swerve = new SwerveDrive(m_gyro, getSwerveModules(), logger);
    driverJoystick = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
    operatorJoystick = new XboxController(OIConstants.BUTTON_BOX_PORT);

    autoPidControllers =
        new AutoPIDControllerContainer(
            new PIDController(
                prefs.getDouble("AutoPosPathP"),
                prefs.getDouble("AutoPosPathI"),
                prefs.getDouble("AutoPosPathD")),
            new PIDController(
                prefs.getDouble("AutoPosPathP"),
                prefs.getDouble("AutoPosPathI"),
                prefs.getDouble("AutoPosPathD")),
            new ProfiledPIDController(
                prefs.getDouble("AutoTurnP"),
                0,
                0,
                new TrapezoidProfile.Constraints(2 * Math.PI, 2 * Math.PI)));

    SmartDashboard.putData(autoPidControllers.xPidController);
    SmartDashboard.putData(autoPidControllers.yPidController);
    SmartDashboard.putData(autoPidControllers.thetaPidController);

    swerve.setDefaultCommand(getNewDefaultSwerveDriveCommand());
    elevator.setDefaultCommand(new DefaultElevator(elevator));
    smartPCM.setDefaultCommand(
        new StartEndCommand(() -> smartPCM.startCompressor(), () -> {}, smartPCM)
            .withName("CompressorCommand"));

    addSubsystems();
    configureButtonBindings();
  }

  protected void addSubsystems() {
    subsystems.add(swerve);
    subsystems.add(camera);
    subsystems.add(smartPCM);
    subsystems.add(elevator);
  }

  protected void configureButtonBindings() {
    Supplier<List<AprilTagPhotonTarget>> visibleCubeTargets =
        () ->
            (camera.hasTargets()
                    && MockBuildWeek2023Constants.cubeTagIds.contains(
                        camera.getBestTarget().get().getId()))
                ? List.of(camera.getBestTarget().get())
                : camera.getTargets().stream()
                    .filter(
                        target -> MockBuildWeek2023Constants.cubeTagIds.contains(target.getId()))
                    .collect(Collectors.toList());
    Supplier<List<AprilTagPhotonTarget>> visibleStationTargets =
        () ->
            (camera.hasTargets()
                    && MockBuildWeek2023Constants.stationTagIds.contains(
                        camera.getBestTarget().get().getId()))
                ? List.of(camera.getBestTarget().get())
                : camera.getTargets().stream()
                    .filter(
                        target -> MockBuildWeek2023Constants.cubeTagIds.contains(target.getId()))
                    .collect(Collectors.toList());

    CommandBase cubeAprilTagCommand = createAprilTagCommand(visibleCubeTargets);
    CommandBase stationAprilTagCommand =
        createAprilTagCommand(
            () ->
                visibleStationTargets.get().stream()
                    .filter(target -> target.getId() == grabbedCubeId)
                    .collect(Collectors.toList()));

    JoystickButton leftBumper =
        new JoystickButton(driverJoystick, XboxController.Button.kLeftBumper.value);
    leftBumper.whenHeld(cubeAprilTagCommand);

    JoystickButton rightBumper =
        new JoystickButton(driverJoystick, XboxController.Button.kRightBumper.value);
    rightBumper.whenHeld(stationAprilTagCommand);

    SmartDashboard.putData(
        "Limelight Snapshot", new InstantCommand(camera::takeSnapshot).withName("Take snapshot"));

    Trigger enabled = new Trigger(RobotState::isEnabled);

    Field2d field = new Field2d();
    SmartDashboard.putData(field);
    SwerveDriveToPosition.setField2d(field);
    swerve.resetOdometry(new Pose2d(4, 2.5, new Rotation2d()));
    enabled.whenActive(new RunCommand(() -> field.setRobotPose(swerve.getOdometry())));
    enabled.whenActive(
        new RunCommand(
            () -> {
              for (AprilTag tag :
                  AprilTagTarget.getVisibleAprilTags(camera, swerve.getOdometry())) {
                field.getObject("AprilTag" + tag.ID).setPose(tag.pose.toPose2d());
              }
            }));

    JoystickButton operatorX = new JoystickButton(operatorJoystick, XboxController.Button.kX.value);
    operatorX.whenPressed(new SnatchCandy(elevator));
    JoystickButton operatorB = new JoystickButton(operatorJoystick, XboxController.Button.kB.value);
    operatorB.whenPressed(new DropCandy(elevator));
    JoystickButton operatorY = new JoystickButton(operatorJoystick, XboxController.Button.kY.value);
    operatorY.whileHeld(new ElevatorUp(elevator));
    SmartDashboard.putData("ElevatorUp", new ElevatorUp(elevator));
    JoystickButton operatorA = new JoystickButton(operatorJoystick, XboxController.Button.kA.value);
    operatorA.whenPressed(new ElevatorDown(elevator));
  }

  private CommandBase createAprilTagCommand(Supplier<List<AprilTagPhotonTarget>> targetSubset) {
    return new ConditionalCommand(
        new DriveAndFaceAprilTag(
            swerve,
            autoPidControllers,
            () -> {
              AprilTagPhotonTarget followedTarget = targetSubset.get().get(0);
              int followedId = followedTarget.getId();
              if (MockBuildWeek2023Constants.cubeTagIds.contains(followedId)) {
                grabbedCubeId = followedId;
                SmartDashboard.putNumber(
                    "Go to station position",
                    MockBuildWeek2023Constants.stationTagIds.indexOf(
                        MockBuildWeek2023Constants.cubeIdToStationId.get(followedId)));
              }
              SmartDashboard.putNumber("Followed AprilTag ID", followedId);
              return followedTarget;
            },
            camera.getCameraPositionConsts().robotCenterToCamera,
            () -> new Transform2d(new Translation2d(0.5, 0), new Rotation2d())),
        new InstantCommand(),
        () -> (targetSubset.get().size() > 0));
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
        getDriverLeftStickInput(),
        () -> squareDeadzone.deadzone(driverJoystick.getRightX()),
        () -> driverJoystick.getStartButtonPressed());
  }

  private Supplier<double[]> getDriverLeftStickInput() {
    return () ->
        circularDeadzone.deadzone(driverJoystick.getLeftY() * -1, driverJoystick.getLeftX() * -1);
  }

  @Override
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
