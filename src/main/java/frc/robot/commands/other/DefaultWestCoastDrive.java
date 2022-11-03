package frc.robot.commands.other;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.WestCoastDriveTrain;
import java.util.function.DoubleSupplier;

public class DefaultWestCoastDrive extends CommandBase {
  private final WestCoastDriveTrain drive;
  private final double speedMultiplier;
  private final DoubleSupplier forward;
  private final DoubleSupplier rot;

  /**
   * Creates a new DefaultWestCoastDrive.
   *
   * @param drive The drive object to run the command on
   * @param speed A speed multiplier for the forward and rotational speeds
   * @param forward The input for the robot's forward/backwards speed
   * @param rot The input for how fast the robot should turn
   *     <p>If you want to use this command in RobotContainer, an example on how to do this with the
   *     WestCoastDriveTrain object of driveTrain (such as WestCoastDriveTrain driveTrain = new
   *     WestCoastDriveTrain();) follows:
   *     <p>new DefaultWestCoastDrive(driveTrain,DriveTrainConstants.DRIVE_TRAIN_SPEED, () ->
   *     driverJoystick.getLeftY(), () -> driverJoystick.getLeftX()));
   */
  public DefaultWestCoastDrive(
      WestCoastDriveTrain drive, double speed, DoubleSupplier forward, DoubleSupplier rot) {
    this.drive = drive;
    speedMultiplier = speed;
    this.forward = forward;
    this.rot = rot;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    drive.driveWithJoysticks(
        forward.getAsDouble() * speedMultiplier, rot.getAsDouble() * speedMultiplier);
  }
}
