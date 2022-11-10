package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Constants.DriveTrainConstants;

public class WestCoastDriveTrain extends WestCoastDriveBase {

  private Talon leftFront;
  private Talon rightFront;
  private Talon leftBack;
  private Talon rightBack;

  private MotorControllerGroup leftMotors;
  private MotorControllerGroup rightMotors;

  private DifferentialDrive drive;

  public WestCoastDriveTrain() {
    leftFront = new Talon(DriveTrainConstants.LEFT_FRONT_PORT);
    leftFront.setInverted(false);
    rightFront = new Talon(DriveTrainConstants.RIGHT_FRONT_PORT);
    rightFront.setInverted(false);
    leftBack = new Talon(DriveTrainConstants.LEFT_BACK_PORT);
    leftBack.setInverted(false);
    rightBack = new Talon(DriveTrainConstants.RIGHT_BACK_PORT);
    rightBack.setInverted(false);

    leftMotors = new MotorControllerGroup(leftFront, leftBack);
    rightMotors = new MotorControllerGroup(rightFront, rightBack);
    drive = new DifferentialDrive(leftMotors, rightMotors);
  }

  @Override
  public void driveWithJoysticks(double forward, double rot) {
    drive.arcadeDrive(forward, rot);
  }

  @Override
  public void onStart() {}

  @Override
  public void onStop() {}

  @Override
  public void zeroSensors() {}

  @Override
  public void updateSmartDashboard() {}

  @Override
  public void setUpTrackables() {}
}
