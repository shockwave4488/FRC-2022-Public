package frc.robot.commands.c2022.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.sensors.NavX;
import frc.lib.sensors.vision.Limelight;
import frc.robot.subsystems.drive.SwerveDrive;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class SwerveDriveWithHeading extends CommandBase {
  private final SwerveDrive swerve;
  private final Limelight limelight;
  private final NavX gyro;
  private final double speedMultiplier;
  private final double rotationMultiplier;
  private final Supplier<double[]> driveValues;
  private final Supplier<double[]> rotValues;
  private final BooleanSupplier fieldRelativeInput;
  private final boolean targetTakeover;
  private boolean fieldRelative = true; // Will change with driver's preference
  private double desiredAngle;

  private final PIDController headingPIDController;
  private static final double HEADING_PID_P = 5;
  private static final double HEADING_PID_I = 0;
  private static final double HEADING_PID_D = 0;

  /**
   * Creates a new DefaultSwerveDrive.
   *
   * @param swerve The swerve subsystem this command will run on
   * @param speedMultiplier A speed multiplier for the driveX & driveY values (input value will be
   *     max speed in meters/sec)
   * @param rotationMultiplier A multiplier for the speed of the robot's rotation (input value will
   *     be max speed in radians/sec)
   * @param driveX The input for the swerve robot's speed on the x axis
   * @param driveY The input for the swerve robot's speed on the y axis
   * @param rot The input for how fast the robot should turn
   * @param fieldRelativeInput The input that indicates if the swerve robot should switch whether it
   *     drives in a field relative manner or robot relative manner
   */
  public SwerveDriveWithHeading(
      SwerveDrive swerve,
      NavX gyro,
      double speedMultiplier,
      double rotationMultiplier,
      Supplier<double[]> driveValues,
      Supplier<double[]> rotValues,
      BooleanSupplier fieldRelativeInput,
      Limelight limelight,
      boolean targetTakeover) {
    this.swerve = swerve;
    this.gyro = gyro;
    this.speedMultiplier = speedMultiplier;
    this.rotationMultiplier = rotationMultiplier;
    this.driveValues = driveValues;
    this.rotValues = rotValues;
    this.fieldRelativeInput = fieldRelativeInput;
    this.limelight = limelight;
    this.targetTakeover = targetTakeover;

    headingPIDController = new PIDController(HEADING_PID_P, HEADING_PID_I, HEADING_PID_D);
    headingPIDController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    desiredAngle = gyro.getYaw().getRadians();
  }

  @Override
  public void execute() {
    fieldRelativeUpdate();
    /*
    The controller's y value effects the drive speed's x value (and vice versa) because the controller input is
    90 degrees off compared to the values SwerveDrive expects (particularly ChassisSpeeds in drive())
    */
    double[] driveSpeedValues = driveValues.get();
    double driveXSpeed = driveSpeedValues[0] * speedMultiplier;
    double driveYSpeed = driveSpeedValues[1] * speedMultiplier;

    double currentAngle = gyro.getYaw().getRadians();
    desiredAngle = calcDesiredAngle(currentAngle);
    SmartDashboard.putNumber("DefaultSD Desired Angle", desiredAngle * 180 / Math.PI);
    SmartDashboard.putNumber("DefaultSD Angle Difference", currentAngle - desiredAngle);
    double rotPower = headingPIDController.calculate(currentAngle, desiredAngle);

    // The rotationMultiplier that's passed in is negative so the values for Math.min and max may
    // seem unintuitive.
    rotPower = Math.min(Math.max(rotationMultiplier, rotPower), -rotationMultiplier);

    SmartDashboard.putNumber("DefaultSD Final Rot Power", rotPower);

    swerve.drive(driveXSpeed, driveYSpeed, rotPower, fieldRelative);
  }

  /**
   * Checks if the value fieldRelativeInput provides has changed, which occurs if the driver has
   * pressed the button the BooleanSupplier is linked to. If the value has changed this class'
   * fieldRelative variable switches to let the drivers toggle between field relative or robot
   * relative operation.
   */
  private void fieldRelativeUpdate() {
    if (fieldRelativeInput.getAsBoolean()) {
      fieldRelative = !fieldRelative;
    }
  }

  private double calcDesiredAngle(double currentAngle) {
    double[] rotSpeedValues = rotValues.get();
    double rotYValue = rotSpeedValues[0];
    double rotXValue = rotSpeedValues[1] * -1;

    if ((rotXValue == 0) && (rotYValue == 0)) {
      return desiredAngle;
    }

    if (rotXValue == 0) {
      rotXValue += 0.0001;
    }

    double theta = Math.atan2(rotYValue, rotXValue);

    theta -= (Math.PI / 2);
    if (theta < -Math.PI) {
      theta += 2 * Math.PI;
    }

    // Rotate by 180 to make it so the intake points the direction of the joystick (because it's the
    // back of the robot)
    theta += Math.PI;
    if (theta > Math.PI) {
      theta -= 2 * Math.PI;
    }

    return theta;
  }

  @Override
  public boolean isFinished() {
    if (targetTakeover) {
      return limelight.hasTargets();
    }
    return false;
  }
}
