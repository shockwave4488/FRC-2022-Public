package frc.robot.commands.c2022.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.sensors.Limelight;
import frc.lib.sensors.NavX;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import java.util.function.Supplier;

public class SwerveTurnToHUB extends CommandBase {
  private final SwerveDrive swerve;
  private final Limelight limelight;
  private final NavX gyro;
  private final double speedMultiplier;
  private final double rotationMultiplier;
  private final Supplier<double[]> driveValues;
  // Below are the same constants as VisionAlignToTarget
  private static final double TURN_PID_P = DriveTrainConstants.SWERVE_DRIVE_ROTATION_P;
  private static final double TURN_PID_I = DriveTrainConstants.SWERVE_DRIVE_ROTATION_I;
  private static final double TURN_PID_D = DriveTrainConstants.SWERVE_DRIVE_ROTATION_D;
  private PIDController turnPID = new PIDController(TURN_PID_P, TURN_PID_I, TURN_PID_D);
  private static final double DONE_RANGE = 6; // degrees
  private static final double MIN_DONE_CYCLES = 5;
  private double doneCycles = 0;
  private static final double CLOSE_ENOUGH_DEGREES = 20;

  private double targetAngle;
  private double currentAngle;

  public SwerveTurnToHUB(
      SwerveDrive swerve,
      Limelight limelight,
      NavX gyro,
      double speedMultiplier,
      double rotationMultiplier,
      Supplier<double[]> driveValues) {
    this.swerve = swerve;
    this.limelight = limelight;
    this.gyro = gyro;
    this.speedMultiplier = speedMultiplier;
    this.rotationMultiplier = rotationMultiplier;
    this.driveValues = driveValues;
    turnPID.enableContinuousInput(-180, 180);
    turnPID.setTolerance(DONE_RANGE);
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    currentAngle = gyro.getYaw().getDegrees();
    targetAngle = calcDesiredAngle();
    doneCycles = 0;
  }

  @Override
  public void execute() {
    currentAngle = gyro.getYaw().getDegrees();
    targetAngle = calcDesiredAngle();
    double rotSpeed = turnPID.calculate(currentAngle, targetAngle);
    rotSpeed *= rotationMultiplier;
    rotSpeed = Math.min(Math.max(-rotationMultiplier, rotSpeed), rotationMultiplier);
    /*
    The controller's y value effects the drive speed's x value (and vice versa) because the controller input is
    90 degrees off compared to the values SwerveDrive expects (particularly ChassisSpeeds in drive())
    */
    double[] driveSpeedValues = driveValues.get();
    double driveXSpeed = driveSpeedValues[0] * speedMultiplier;
    double driveYSpeed = driveSpeedValues[1] * speedMultiplier;
    swerve.drive(
        driveXSpeed, driveYSpeed, rotSpeed, true); // Robot relative and hub centric do not mix.

    if (turnPID.atSetpoint()) {
      doneCycles++;
    } else {
      doneCycles = 0;
    }
  }

  private double calcDesiredAngle() {
    Pose2d currentPose = swerve.getOdometry();
    double currentX = currentPose.getX();
    double currentY = currentPose.getY();
    double relativeX = Constants.FieldConstants.HUB_CENTER.getX() - currentX;
    double relativeY = Constants.FieldConstants.HUB_CENTER.getY() - currentY;
    double desiredAngle;
    if (relativeX == 0) {
      relativeX += 0.0001;
    }
    desiredAngle = Math.atan2(relativeY, relativeX);
    desiredAngle *= 180 / Math.PI; // convert to degrees
    return desiredAngle;
  }

  // Not removing this again in case we need to use it again.
  private boolean closeEnoughAndSeesTarget() {
    return ((Math.abs(currentAngle - targetAngle) < CLOSE_ENOUGH_DEGREES) && limelight.hasTarget());
  }

  public boolean isFinished() {
    return doneCycles > MIN_DONE_CYCLES;
  }
}
